import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';

const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
const EXPECTED_MUJOCO_VERSION = '3.11.0';
const MAX_STEP_BATCH = 100000;
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;

let mujoco = null; let model = null; let data = null; let paused = false; let descriptor = null;
let jointState = new Map(); let actuatorState = new Map(); let bodyState = new Map(); let modelInfo = null;

function reply(id, ok, payload = null, error = null) { postMessage({ id, ok, payload, error }); }
async function ensureMuJoCo() { if (mujoco) return mujoco; mujoco = await loadMujoco({ locateFile: (path) => new URL(path, MUJOCO_BASE_URL).href }); return mujoco; }
async function sha256Text(text) { if (!crypto?.subtle) throw new Error('Web Crypto is required to identify model bytes'); const digest = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text)); return [...new Uint8Array(digest)].map((value) => value.toString(16).padStart(2, '0')).join(''); }
function enumValue(name) { const entry = mujoco?.mjtObj?.[name]; const value = entry && typeof entry === 'object' && 'value' in entry ? entry.value : entry; if (!Number.isInteger(Number(value))) throw new Error(`MuJoCo object enum ${name} is unavailable`); return Number(value); }
function idFor(typeName, name) { const id = Number(mujoco.mj_name2id(model, enumValue(typeName), name)); if (!Number.isInteger(id) || id < 0) throw new Error(`Required MuJoCo ${typeName} named ${name} is missing`); return id; }
function nameForGeom(id) { if (!Number.isInteger(id) || id < 0 || typeof mujoco?.mj_id2name !== 'function') return null; try { return mujoco.mj_id2name(model, enumValue('mjOBJ_GEOM'), id) || null; } catch { return null; } }
function runtimeVersionEvidence() { if (typeof mujoco?.mj_versionString !== 'function') return { version: EXPECTED_MUJOCO_VERSION, evidence: 'bundled asset manifest and repository hash gate' }; const version = String(mujoco.mj_versionString()); if (version !== EXPECTED_MUJOCO_VERSION) throw new Error(`Bundled MuJoCo runtime reports ${version}; expected ${EXPECTED_MUJOCO_VERSION}`); return { version, evidence: 'mj_versionString runtime introspection' }; }
function validatedModelUrl(asset) { if (!MODEL_ASSET_RE.test(asset || '') || String(asset).includes('..')) throw new Error('Worker rejected non-registry model asset path'); const url = new URL(`../../${asset}`, import.meta.url); if (url.origin !== self.location.origin) throw new Error('Worker model asset must be same-origin'); return url.href; }

function resolveModelAddresses(modelSha256) {
  jointState = new Map(); actuatorState = new Map(); bodyState = new Map();
  for (const joint of descriptor.joints) {
    const id = idFor('mjOBJ_JOINT', joint.id); const qpos = Number(model.jnt_qposadr[id]); const dof = Number(model.jnt_dofadr[id]);
    if (![qpos, dof].every(Number.isInteger)) throw new Error(`MuJoCo address table is invalid for joint ${joint.id}`);
    const offset = id * 2; jointState.set(joint.id, { id, qpos, dof, range: [Number(model.jnt_range[offset]), Number(model.jnt_range[offset + 1])] });
  }
  for (const actuator of descriptor.actuators) {
    const id = idFor('mjOBJ_ACTUATOR', actuator.id); const joint = jointState.get(actuator.jointId);
    if (!joint) throw new Error(`Actuator ${actuator.id} references unknown descriptor joint ${actuator.jointId}`);
    if (model.actuator_trnid?.length && Number(model.actuator_trnid[id * 2]) !== joint.id) throw new Error(`Actuator ${actuator.id} is not mapped to joint ${actuator.jointId}`);
    const offset = id * 2; actuatorState.set(actuator.id, { ...actuator, id, controlRange: [Number(model.actuator_ctrlrange[offset]), Number(model.actuator_ctrlrange[offset + 1])] });
  }
  for (const body of descriptor.bodies || []) bodyState.set(body.id, { id: idFor('mjOBJ_BODY', body.id) });
  const timestepSeconds = Number(model.opt?.timestep);
  if (!Number.isFinite(timestepSeconds) || Math.abs(timestepSeconds - Number(descriptor.physics.timestepSeconds)) > 1e-12) throw new Error(`Unexpected timestep ${timestepSeconds}; expected ${descriptor.physics.timestepSeconds}`);
  const version = runtimeVersionEvidence(); modelInfo = { modelSha256, engineVersion: version.version, engineVersionEvidence: version.evidence, timestepSeconds };
}

function readContacts() {
  const count = Number(data?.ncon || 0); const contacts = []; let readable = true; const collection = data?.contact;
  if (!collection) return { count, readable: count === 0, contacts };
  try {
    const available = typeof collection.size === 'function' ? Number(collection.size()) : count; if (available < count) readable = false;
    for (let index = 0; index < Math.min(count, available); index += 1) {
      const contact = typeof collection.get === 'function' ? collection.get(index) : collection[index]; if (!contact) { readable = false; continue; }
      try { const geom1 = Number(contact.geom?.[0] ?? contact.geom1 ?? -1); const geom2 = Number(contact.geom?.[1] ?? contact.geom2 ?? -1); contacts.push({ geom1, geom2, geom1Name: nameForGeom(geom1), geom2Name: nameForGeom(geom2), distanceM: Number(contact.dist ?? 0) }); } finally { contact.delete?.(); }
    }
  } finally { collection.delete?.(); }
  return { count, readable, contacts };
}

function actuatorForJoint(jointId) { const matches = [...actuatorState.values()].filter((actuator) => actuator.jointId === jointId && actuator.command === 'position-rad'); if (matches.length !== 1) throw new Error(`Joint ${jointId} does not have exactly one position actuator`); return matches[0]; }
function observation() {
  if (!model || !data || !descriptor || !modelInfo) return { simulationTime: 0, model: null, engine: null, joints: {}, bodies: {}, contactCount: 0, contactsReadable: false, contacts: [] };
  const joints = {}; for (const [name, joint] of jointState) { const actuator = actuatorForJoint(name); joints[name] = { positionRad: Number(data.qpos[joint.qpos]), velocityRadS: Number(data.qvel[joint.dof]), targetRad: Number(data.ctrl[actuator.id]), controlRangeRad: [...actuator.controlRange], jointRangeRad: [...joint.range] }; }
  const bodies = {}; for (const [name, body] of bodyState) { const offset = body.id * 3; bodies[name] = { frame: 'mujoco_world', positionM: [Number(data.xpos[offset]), Number(data.xpos[offset + 1]), Number(data.xpos[offset + 2])] }; }
  const contactState = readContacts();
  return { simulationTime: Number(data.time || 0), model: { id: descriptor.modelId || descriptor.id, asset: descriptor.asset, sha256: modelInfo.modelSha256 }, engine: { version: modelInfo.engineVersion, versionEvidence: modelInfo.engineVersionEvidence, timestepSeconds: modelInfo.timestepSeconds }, joints, bodies, contactCount: contactState.count, contactsReadable: contactState.readable, contacts: contactState.contacts };
}

function disposeModel() { try { data?.delete?.(); } catch {} try { model?.delete?.(); } catch {} data = null; model = null; descriptor = null; jointState = new Map(); actuatorState = new Map(); bodyState = new Map(); modelInfo = null; paused = false; }
async function load(modelPackage) {
  if (!modelPackage?.id || !modelPackage?.asset || !modelPackage?.sha256 || !Array.isArray(modelPackage.joints) || !Array.isArray(modelPackage.actuators)) throw new Error('Worker requires a validated registered model package descriptor');
  const modelUrl = validatedModelUrl(modelPackage.asset); const mj = await ensureMuJoCo(); disposeModel();
  const xml = await fetch(modelUrl, { cache: 'no-store' }).then((response) => { if (!response.ok) throw new Error(`MuJoCo model returned HTTP ${response.status}`); return response.text(); });
  const modelSha256 = await sha256Text(xml); if (modelSha256 !== modelPackage.sha256) throw new Error(`Model SHA-256 mismatch for ${modelPackage.id}`);
  model = mj.from_xml_string(xml); if (!model) throw new Error(`MuJoCo failed to compile ${modelPackage.id}`); data = new mj.MjData(model); if (!data) throw new Error(`MuJoCo failed to allocate data for ${modelPackage.id}`);
  descriptor = structuredClone(modelPackage); resolveModelAddresses(modelSha256); paused = false; mj.mj_forward(model, data); return observation();
}
function reset() { if (!model || !data) throw new Error('No MuJoCo model is loaded'); mujoco.mj_resetData(model, data); paused = false; mujoco.mj_forward(model, data); return observation(); }
function step(count = 1) { if (!model || !data) throw new Error('No MuJoCo model is loaded'); if (paused) return observation(); if (!Number.isInteger(count) || count < 1 || count > MAX_STEP_BATCH) throw new RangeError(`step count must be an integer from 1 to ${MAX_STEP_BATCH}`); for (let index = 0; index < count; index += 1) mujoco.mj_step(model, data); return observation(); }
function command(payload = {}) {
  if (!model || !data || !descriptor || !modelInfo) throw new Error('No MuJoCo model is loaded'); if (payload.type !== 'set_joint_target') throw new Error(`Unsupported physical command: ${payload.type}`);
  const jointId = payload.jointId || (descriptor.joints.length === 1 ? descriptor.joints[0].id : null); if (!jointId || !jointState.has(jointId)) throw new Error('set_joint_target requires a declared jointId for this model');
  const actuator = actuatorForJoint(jointId); const target = Number(payload.targetRad); if (!Number.isFinite(target)) throw new TypeError('targetRad must be finite'); const [minimum, maximum] = actuator.controlRange;
  if (target < minimum || target > maximum) throw new RangeError(`targetRad ${target} is outside the actuator control range ${minimum}..${maximum}`);
  const jointRange = jointState.get(jointId).range; if (jointRange.every(Number.isFinite) && (target < jointRange[0] || target > jointRange[1])) throw new RangeError(`targetRad ${target} is outside joint ${jointId} range ${jointRange[0]}..${jointRange[1]}`);
  data.ctrl[actuator.id] = target; return observation();
}

self.onmessage = async (event) => {
  const { id, op, payload } = event.data || {};
  try { let result; if (op === 'load') result = await load(payload?.modelPackage); else if (op === 'reset') result = reset(); else if (op === 'step') result = step(payload?.count); else if (op === 'observe') result = observation(); else if (op === 'command') result = command(payload); else if (op === 'pause') { paused = true; result = observation(); } else if (op === 'resume') { paused = false; result = observation(); } else if (op === 'dispose') { disposeModel(); result = true; } else throw new Error(`Unknown worker operation: ${op}`); reply(id, true, result); }
  catch (error) { reply(id, false, null, String(error?.stack || error?.message || error)); }
};
