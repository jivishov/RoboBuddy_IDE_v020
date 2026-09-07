import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';

const DEFAULT_MODEL_URL = new URL('../../models/vertical-slice/model.xml', import.meta.url).href;
const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
const EXPECTED_TIMESTEP_SECONDS = 0.002;
const MAX_STEP_BATCH = 100000;

let mujoco = null;
let model = null;
let data = null;
let paused = false;
let ids = null;
let addresses = null;
let modelInfo = null;

function reply(id, ok, payload = null, error = null) {
  postMessage({ id, ok, payload, error });
}

async function ensureMuJoCo() {
  if (mujoco) return mujoco;
  mujoco = await loadMujoco({ locateFile: (path) => new URL(path, MUJOCO_BASE_URL).href });
  return mujoco;
}

function enumValue(name) {
  const entry = mujoco?.mjtObj?.[name];
  const value = entry && typeof entry === 'object' && 'value' in entry ? entry.value : entry;
  if (!Number.isInteger(Number(value))) throw new Error(`MuJoCo object enum ${name} is unavailable`);
  return Number(value);
}

function idFor(typeName, name) {
  const id = Number(mujoco.mj_name2id(model, enumValue(typeName), name));
  if (!Number.isInteger(id) || id < 0) throw new Error(`Required MuJoCo ${typeName} named ${name} is missing`);
  return id;
}

function nameForGeom(id) {
  if (!Number.isInteger(id) || id < 0 || typeof mujoco?.mj_id2name !== 'function') return null;
  try { return mujoco.mj_id2name(model, enumValue('mjOBJ_GEOM'), id) || null; }
  catch { return null; }
}

function resolveModelAddresses() {
  ids = {
    hinge: idFor('mjOBJ_JOINT', 'hinge'),
    actuator: idFor('mjOBJ_ACTUATOR', 'hinge_position'),
    freeBoxBody: idFor('mjOBJ_BODY', 'free_box'),
  };
  addresses = {
    hingeQpos: Number(model.jnt_qposadr[ids.hinge]),
    hingeDof: Number(model.jnt_dofadr[ids.hinge]),
  };
  if (![addresses.hingeQpos, addresses.hingeDof].every(Number.isInteger)) {
    throw new Error('MuJoCo joint address table is invalid');
  }
  if (model.actuator_trnid?.length) {
    const actuatorJoint = Number(model.actuator_trnid[ids.actuator * 2]);
    if (actuatorJoint !== ids.hinge) throw new Error('hinge_position actuator is not mapped to hinge');
  }
  const ctrlOffset = ids.actuator * 2;
  const controlRange = [Number(model.actuator_ctrlrange[ctrlOffset]), Number(model.actuator_ctrlrange[ctrlOffset + 1])];
  const jointRangeOffset = ids.hinge * 2;
  const jointRange = [Number(model.jnt_range[jointRangeOffset]), Number(model.jnt_range[jointRangeOffset + 1])];
  const timestepSeconds = Number(model.opt?.timestep);
  if (!Number.isFinite(timestepSeconds) || Math.abs(timestepSeconds - EXPECTED_TIMESTEP_SECONDS) > 1e-12) {
    throw new Error(`Unexpected Phase 1 timestep ${timestepSeconds}; expected ${EXPECTED_TIMESTEP_SECONDS}`);
  }
  modelInfo = {
    engineVersion: typeof mujoco.mj_versionString === 'function' ? String(mujoco.mj_versionString()) : null,
    timestepSeconds,
    controlRange,
    jointRange,
  };
}

function readContacts() {
  const count = Number(data?.ncon || 0);
  const contacts = [];
  let readable = true;
  const collection = data?.contact;
  if (!collection) return { count, readable: count === 0, contacts };
  try {
    const available = typeof collection.size === 'function' ? Number(collection.size()) : count;
    if (available < count) readable = false;
    for (let index = 0; index < Math.min(count, available); index += 1) {
      const contact = typeof collection.get === 'function' ? collection.get(index) : collection[index];
      if (!contact) {
        readable = false;
        continue;
      }
      try {
        const geom1 = Number(contact.geom?.[0] ?? contact.geom1 ?? -1);
        const geom2 = Number(contact.geom?.[1] ?? contact.geom2 ?? -1);
        contacts.push({
          geom1,
          geom2,
          geom1Name: nameForGeom(geom1),
          geom2Name: nameForGeom(geom2),
          distanceM: Number(contact.dist ?? 0),
        });
      } finally {
        contact.delete?.();
      }
    }
  } finally {
    collection.delete?.();
  }
  return { count, readable, contacts };
}

function observation() {
  if (!model || !data || !ids || !addresses || !modelInfo) {
    return { simulationTime: 0, engine: null, joints: {}, bodies: {}, contactCount: 0, contactsReadable: false, contacts: [] };
  }
  const bodyOffset = ids.freeBoxBody * 3;
  const contactState = readContacts();
  return {
    simulationTime: Number(data.time || 0),
    engine: {
      version: modelInfo.engineVersion,
      timestepSeconds: modelInfo.timestepSeconds,
    },
    joints: {
      hinge: {
        positionRad: Number(data.qpos[addresses.hingeQpos]),
        velocityRadS: Number(data.qvel[addresses.hingeDof]),
        targetRad: Number(data.ctrl[ids.actuator]),
        controlRangeRad: [...modelInfo.controlRange],
        jointRangeRad: [...modelInfo.jointRange],
      },
    },
    bodies: {
      free_box: {
        positionM: [
          Number(data.xpos[bodyOffset]),
          Number(data.xpos[bodyOffset + 1]),
          Number(data.xpos[bodyOffset + 2]),
        ],
      },
    },
    contactCount: contactState.count,
    contactsReadable: contactState.readable,
    contacts: contactState.contacts,
  };
}

function disposeModel() {
  try { data?.delete?.(); } catch {}
  try { model?.delete?.(); } catch {}
  data = null;
  model = null;
  ids = null;
  addresses = null;
  modelInfo = null;
  paused = false;
}

async function load(url = DEFAULT_MODEL_URL) {
  const requestedUrl = new URL(url, self.location.href).href;
  if (requestedUrl !== DEFAULT_MODEL_URL) throw new Error('Phase 1 worker only permits the pinned vertical-slice model');
  const mj = await ensureMuJoCo();
  disposeModel();
  const xml = await fetch(DEFAULT_MODEL_URL, { cache: 'no-store' }).then((response) => {
    if (!response.ok) throw new Error(`MuJoCo model returned HTTP ${response.status}`);
    return response.text();
  });
  model = mj.from_xml_string(xml);
  if (!model) throw new Error('MuJoCo failed to compile the Phase 1 model');
  data = new mj.MjData(model);
  if (!data) throw new Error('MuJoCo failed to allocate Phase 1 data');
  resolveModelAddresses();
  paused = false;
  mj.mj_forward(model, data);
  return observation();
}

function reset() {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  mujoco.mj_resetData(model, data);
  paused = false;
  mujoco.mj_forward(model, data);
  return observation();
}

function step(count = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (paused) return observation();
  if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) {
    throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`);
  }
  for (let index = 0; index < count; index += 1) mujoco.mj_step(model, data);
  return observation();
}

function command(payload = {}) {
  if (!model || !data || !ids || !modelInfo) throw new Error('No MuJoCo model is loaded');
  if (payload.type !== 'set_joint_target') throw new Error(`Unsupported Phase 1 command: ${payload.type}`);
  const target = Number(payload.targetRad);
  if (!Number.isFinite(target)) throw new TypeError('targetRad must be finite');
  const [minimum, maximum] = modelInfo.controlRange;
  if (target < minimum || target > maximum) {
    throw new RangeError(`targetRad ${target} is outside the actuator control range ${minimum}..${maximum}`);
  }
  data.ctrl[ids.actuator] = target;
  return observation();
}

self.onmessage = async (event) => {
  const { id, op, payload } = event.data || {};
  try {
    let result;
    if (op === 'load') result = await load(payload?.modelUrl);
    else if (op === 'reset') result = reset();
    else if (op === 'step') result = step(payload?.count);
    else if (op === 'observe') result = observation();
    else if (op === 'command') result = command(payload);
    else if (op === 'pause') { paused = true; result = observation(); }
    else if (op === 'resume') { paused = false; result = observation(); }
    else if (op === 'dispose') { disposeModel(); result = true; }
    else throw new Error(`Unknown worker operation: ${op}`);
    reply(id, true, result);
  } catch (error) {
    reply(id, false, null, String(error?.stack || error?.message || error));
  }
};
