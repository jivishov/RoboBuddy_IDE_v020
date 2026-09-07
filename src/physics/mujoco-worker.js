import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';

const DEFAULT_MODEL_URL = new URL('../../models/vertical-slice/model.xml', import.meta.url).href;
const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);

let mujoco = null;
let model = null;
let data = null;
let paused = false;
let modelUrl = DEFAULT_MODEL_URL;

function reply(id, ok, payload = null, error = null) {
  postMessage({ id, ok, payload, error });
}

async function ensureMuJoCo() {
  if (mujoco) return mujoco;
  mujoco = await loadMujoco({ locateFile: (path) => new URL(path, MUJOCO_BASE_URL).href });
  return mujoco;
}

function disposeModel() {
  try { data?.delete?.(); } catch {}
  try { model?.delete?.(); } catch {}
  data = null;
  model = null;
}

function observation() {
  if (!model || !data) return { simulationTime: 0, joints: {}, bodies: {}, contacts: [] };
  const contacts = [];
  const ncon = Number(data.ncon || 0);
  for (let i = 0; i < ncon; i += 1) {
    const c = data.contact?.get ? data.contact.get(i) : null;
    if (!c) continue;
    contacts.push({ geom1: Number(c.geom1), geom2: Number(c.geom2), distance: Number(c.dist ?? 0) });
  }
  return {
    simulationTime: Number(data.time || 0),
    joints: {
      hinge: {
        positionRad: Number(data.qpos?.[0] ?? 0),
        velocityRadS: Number(data.qvel?.[0] ?? 0),
        targetRad: data.ctrl?.length ? Number(data.ctrl[0]) : null,
      },
    },
    bodies: {
      free_box: {
        positionM: [
          Number(data.qpos?.[1] ?? 0),
          Number(data.qpos?.[2] ?? 0),
          Number(data.qpos?.[3] ?? 0),
        ],
      },
    },
    contacts,
  };
}

async function load(url = DEFAULT_MODEL_URL) {
  const mj = await ensureMuJoCo();
  disposeModel();
  modelUrl = new URL(url, self.location.href).href;
  const xml = await fetch(modelUrl, { cache: 'no-store' }).then((r) => {
    if (!r.ok) throw new Error(`MuJoCo model returned HTTP ${r.status}`);
    return r.text();
  });
  model = mj.from_xml_string(xml);
  data = new mj.MjData(model);
  paused = false;
  mj.mj_forward(model, data);
  return observation();
}

function step(count = 1) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (paused) return observation();
  const steps = Math.max(0, Math.min(100000, Math.trunc(Number(count) || 0)));
  for (let i = 0; i < steps; i += 1) mujoco.mj_step(model, data);
  return observation();
}

function command(payload = {}) {
  if (!model || !data) throw new Error('No MuJoCo model is loaded');
  if (payload.type === 'set_joint_target') {
    const target = Number(payload.targetRad);
    if (!Number.isFinite(target)) throw new TypeError('targetRad must be finite');
    data.ctrl[0] = Math.max(-1.2, Math.min(1.2, target));
    return observation();
  }
  throw new Error(`Unsupported Phase 1 command: ${payload.type}`);
}

self.onmessage = async (event) => {
  const { id, op, payload } = event.data || {};
  try {
    let result;
    if (op === 'load') result = await load(payload?.modelUrl);
    else if (op === 'reset') result = await load(modelUrl);
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
