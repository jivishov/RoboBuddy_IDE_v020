import { OpenArmServo, OPENARM_CONTROL_PROFILE } from './openarm-servo.js';
import { solveOpenArmIK } from './openarm-ik.js';
import { stripOpenArmReferenceWorkcellXml } from './openarm-lab-builder.js';
import { validateOpenArmLabEquipment, appendOpenArmLabEquipmentXml } from './openarm-lab-equipment.js';
import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';
import { MAX_ADVANCE_STEPS_PER_REQUEST, MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE, sampledObservationCount } from './backend-contract.js';

const MUJOCO_BASE_URL = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
const EXPECTED_MUJOCO_VERSION = '3.11.0';
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;
const INTEGRATOR_CODES = Object.freeze({ Euler: 0, RK4: 1, implicit: 2, implicitfast: 3 });
const PATH_LIMITS = Object.freeze({ waypoints: 16, samples: 900, minSeparationM: .0005, maxSeparationM: .03, minJointStepRad: .01, maxJointStepRad: .15 });

let mujoco = null, model = null, data = null, descriptor = null, modelInfo = null, servo = null, equipment = null, motionPlan = null, paused = false;
let jointState = new Map(), couplingState = new Map(), actuatorState = new Map(), bodyState = new Map();
const reply = (id, ok, payload = null, error = null) => postMessage({ id, ok, payload, error });
async function ensureMuJoCo() { if (!mujoco) mujoco = await loadMujoco({ locateFile: path => new URL(path, MUJOCO_BASE_URL).href }); return mujoco; }
async function sha256Text(text) { const digest = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(text)); return [...new Uint8Array(digest)].map(v => v.toString(16).padStart(2, '0')).join(''); }
function enumValue(name) { const entry = mujoco?.mjtObj?.[name], value = entry && typeof entry === 'object' && 'value' in entry ? entry.value : entry; if (!Number.isInteger(Number(value))) throw new Error(`MuJoCo object enum ${name} unavailable`); return Number(value); }
function idFor(type, name) { const id = Number(mujoco.mj_name2id(model, enumValue(type), name)); if (!Number.isInteger(id) || id < 0) throw new Error(`Required MuJoCo ${type} ${name} missing`); return id; }
function nameForGeom(id) { if (!Number.isInteger(id) || id < 0 || typeof mujoco?.mj_id2name !== 'function') return null; try { return mujoco.mj_id2name(model, enumValue('mjOBJ_GEOM'), id) || null; } catch { return null; } }
function validatedModelUrl(asset) { if (!MODEL_ASSET_RE.test(asset || '') || String(asset).includes('..')) throw new Error('Lab Builder rejected model path'); const url = new URL(`../../${asset}`, import.meta.url); if (url.origin !== self.location.origin) throw new Error('Model asset must be same-origin'); return url.href; }
function closeArray(actual, expected, label, tol = 1e-9) { if (!Array.isArray(expected) || actual.length !== expected.length) throw new Error(`${label} shape mismatch`); actual.forEach((v, i) => { if (!Number.isFinite(v) || Math.abs(v - Number(expected[i])) > tol) throw new Error(`${label} mismatch at ${i}`); }); }
function runtimeVersion() { const version = typeof mujoco?.mj_versionString === 'function' ? String(mujoco.mj_versionString()) : EXPECTED_MUJOCO_VERSION; if (version !== EXPECTED_MUJOCO_VERSION) throw new Error(`Bundled MuJoCo ${version}; expected ${EXPECTED_MUJOCO_VERSION}`); return version; }

function resolveAddresses(modelSha256) {
  jointState = new Map(); couplingState = new Map(); actuatorState = new Map(); bodyState = new Map();
  for (const joint of descriptor.joints) {
    const id = idFor('mjOBJ_JOINT', joint.id), qpos = Number(model.jnt_qposadr[id]), dof = Number(model.jnt_dofadr[id]);
    const range = [Number(model.jnt_range[id * 2]), Number(model.jnt_range[id * 2 + 1])];
    const axis = [Number(model.jnt_axis[id * 3]), Number(model.jnt_axis[id * 3 + 1]), Number(model.jnt_axis[id * 3 + 2])];
    if (joint.rangeRad) closeArray(range, joint.rangeRad, `${joint.id} range`);
    if (joint.axis) closeArray(axis, joint.axis, `${joint.id} axis`);
    jointState.set(joint.id, { id, qpos, dof, range, axis });
  }
  for (const coupling of descriptor.mechanicalCouplings || []) {
    const driver = jointState.get(coupling.driverJointId); if (!driver) throw new Error(`Unknown coupling driver ${coupling.driverJointId}`);
    const id = idFor('mjOBJ_JOINT', coupling.followerJointId), qpos = Number(model.jnt_qposadr[id]), dof = Number(model.jnt_dofadr[id]);
    const range = [Number(model.jnt_range[id * 2]), Number(model.jnt_range[id * 2 + 1])];
    const axis = [Number(model.jnt_axis[id * 3]), Number(model.jnt_axis[id * 3 + 1]), Number(model.jnt_axis[id * 3 + 2])];
    couplingState.set(coupling.id, { ...coupling, driver, id, qpos, dof, range, axis });
    jointState.set(coupling.followerJointId, { id, qpos, dof, range, axis, passiveCoupled: true });
  }
  for (const actuator of descriptor.actuators) {
    const id = idFor('mjOBJ_ACTUATOR', actuator.id), joint = jointState.get(actuator.jointId); if (!joint) throw new Error(`Unknown actuator joint ${actuator.jointId}`);
    if (model.actuator_trnid?.length && Number(model.actuator_trnid[id * 2]) !== joint.id) throw new Error(`Actuator ${actuator.id} is not mapped to ${actuator.jointId}`);
    const controlRange = [Number(model.actuator_ctrlrange[id * 2]), Number(model.actuator_ctrlrange[id * 2 + 1])];
    if (actuator.controlRangeRad) closeArray(controlRange, actuator.controlRangeRad, `${actuator.id} control range`);
    actuatorState.set(actuator.id, { ...actuator, id, controlRange });
  }
  for (const body of descriptor.bodies || []) {
    const id = idFor('mjOBJ_BODY', body.id); let freeDof = null;
    if (body.freeJointId) { const j = idFor('mjOBJ_JOINT', body.freeJointId); freeDof = Number(model.jnt_dofadr[j]); if (!Number.isInteger(freeDof) || freeDof < 0) throw new Error(`Invalid free joint for ${body.id}`); }
    bodyState.set(body.id, { id, freeDof });
  }
  const timestepSeconds = Number(model.opt?.timestep), expected = Number(descriptor.physics.timestepSeconds);
  if (!Number.isFinite(timestepSeconds) || Math.abs(timestepSeconds - expected) > 1e-12) throw new Error(`Unexpected timestep ${timestepSeconds}`);
  if (Number(model.opt?.integrator) !== INTEGRATOR_CODES[descriptor.physics.integrator]) throw new Error('Unexpected integrator');
  modelInfo = { modelSha256, engineVersion: runtimeVersion(), timestepSeconds };
}

function positionActuator(jointId, required = false) { const matches = [...actuatorState.values()].filter(a => a.jointId === jointId && a.command === 'position-rad'); if (matches.length > 1 || (required && matches.length !== 1)) throw new Error(`Joint ${jointId} has invalid actuator mapping`); return matches[0] || null; }
function contacts() {
  const count = Number(data?.ncon || 0); if (count > 512) throw new RangeError('Lab Builder contact budget exceeded');
  const result = []; if (!data?.contact) return { count, readable: count === 0, contacts: result };
  const force = new mujoco.DoubleBuffer(6), collection = data.contact; let readable = true;
  try {
    const available = typeof collection.size === 'function' ? Number(collection.size()) : count; if (available < count) readable = false;
    for (let i = 0; i < Math.min(count, available); i += 1) {
      const c = typeof collection.get === 'function' ? collection.get(i) : collection[i]; if (!c) { readable = false; continue; }
      try {
        const geom1 = Number(c.geom?.[0] ?? c.geom1 ?? -1), geom2 = Number(c.geom?.[1] ?? c.geom2 ?? -1);
        mujoco.mj_contactForce(model, data, i, force);
        result.push({ geom1, geom2, geom1Name: nameForGeom(geom1), geom2Name: nameForGeom(geom2), distanceM: Number(c.dist ?? 0), normalForceN: Math.max(0, Number(force.GetView()[0])), positionM: Array.from(c.pos || []).slice(0, 3) });
      } finally { c.delete?.(); }
    }
  } finally { force.delete(); collection.delete?.(); }
  return { count, readable, contacts: result };
}
function observation() {
  if (!model || !data) return { simulationTime: 0, joints: {}, bodies: {}, contactCount: 0, contactsReadable: false, contacts: [] };
  const joints = {};
  for (const [name, joint] of jointState) {
    const actuator = positionActuator(name);
    joints[name] = { positionRad: Number(data.qpos[joint.qpos]), velocityRadS: Number(data.qvel[joint.dof]), targetRad: actuator ? servo?.target(name) ?? Number(data.ctrl[actuator.id]) : null, referenceRad: actuator ? servo?.reference(name) ?? Number(data.ctrl[actuator.id]) : null, actuatorTargetRad: actuator ? Number(data.ctrl[actuator.id]) : null, effortNm: actuator ? Number(data.actuator_force?.[actuator.id] ?? 0) : null, controlRangeRad: actuator ? [...actuator.controlRange] : null, jointRangeRad: [...joint.range] };
  }
  const bodies = {};
  for (const [name, body] of bodyState) {
    const p = body.id * 3, q = body.id * 4;
    const record = { frame: 'mujoco_world', positionM: [Number(data.xpos[p]), Number(data.xpos[p + 1]), Number(data.xpos[p + 2])], quaternionWxyz: [Number(data.xquat[q]), Number(data.xquat[q + 1]), Number(data.xquat[q + 2]), Number(data.xquat[q + 3])] };
    if (body.freeDof != null) { record.linearVelocityFrame = 'mujoco_world'; record.angularVelocityFrame = 'body_local'; record.linearVelocityMS = [0, 1, 2].map(i => Number(data.qvel[body.freeDof + i])); record.angularVelocityRadS = [3, 4, 5].map(i => Number(data.qvel[body.freeDof + i])); }
    bodies[name] = record;
  }
  const c = contacts(), pinchReferences = {};
  for (const side of ['left', 'right']) { const site = idFor('mjOBJ_SITE', `${side}_pinch_reference`), ee = bodies[`openarm_${side}_ee_base_link`]; pinchReferences[side] = { frame: 'mujoco_world', positionM: Array.from(data.site_xpos.slice(site * 3, site * 3 + 3)), quaternionWxyz: ee.quaternionWxyz }; }
  return { simulationTime: Number(data.time || 0), model: { id: descriptor.modelId || descriptor.id, asset: descriptor.asset, sha256: modelInfo.modelSha256 }, engine: { version: modelInfo.engineVersion, timestepSeconds: modelInfo.timestepSeconds }, joints, bodies, contactCount: c.count, contactsReadable: c.readable, contacts: c.contacts, openarm: { workspaceMode: 'lab_builder', pinchReferences, observationPhase: 'post-step forward dynamics', controlProfile: OPENARM_CONTROL_PROFILE, motionPlan, equipment: equipment?.records || [], equipmentJoints: [] }, setupLog: descriptor.equipment?.length ? [{ type: 'explicit-lab-builder-compile-and-reset', simulationTimeSeconds: 0, equipmentIds: descriptor.equipment.map(e => e.id), modelSha256: modelInfo.modelSha256 }] : [] };
}

function applyInitial() {
  if (!model || !data || !descriptor) throw new Error('No Lab Builder model loaded');
  mujoco.mj_resetData(model, data);
  for (const [jointId, raw] of Object.entries(descriptor.initialJointPositionsRad || {})) {
    const joint = jointState.get(jointId), value = Number(raw); if (!joint || !Number.isFinite(value)) throw new Error(`Invalid initial joint ${jointId}`);
    if (joint.range.every(Number.isFinite) && (value < joint.range[0] || value > joint.range[1])) throw new RangeError(`Initial ${jointId} outside joint range`);
    const actuator = positionActuator(jointId); if (actuator && (value < actuator.controlRange[0] || value > actuator.controlRange[1])) throw new RangeError(`Initial ${jointId} outside actuator range`);
    data.qpos[joint.qpos] = value; if (actuator) data.ctrl[actuator.id] = value;
  }
  for (const coupling of couplingState.values()) {
    const value = Number(data.qpos[coupling.driver.qpos]) * Number(coupling.multiplier ?? 1) + Number(coupling.offsetRad ?? 0);
    if (!Number.isFinite(value) || (coupling.range.every(Number.isFinite) && (value < coupling.range[0] || value > coupling.range[1]))) throw new RangeError(`Invalid initial mechanical coupling ${coupling.id}`);
    data.qpos[coupling.qpos] = value;
  }
  paused = false; mujoco.mj_forward(model, data); servo = new OpenArmServo(model, data, actuatorState, jointState); servo.apply(); mujoco.mj_forward(model, data); motionPlan = null; return observation();
}
function dispose() { try { data?.delete?.(); } catch {} try { model?.delete?.(); } catch {} data = model = descriptor = modelInfo = servo = equipment = motionPlan = null; jointState = new Map(); couplingState = new Map(); actuatorState = new Map(); bodyState = new Map(); paused = false; }
function validateClearance() {
  if (!equipment) return;
  const names = Array.from({ length: Number(model.ngeom) }, (_, i) => nameForGeom(i));
  const owner = name => equipment.records.find(e => e.geometryIds.includes(name))?.id;
  for (let a = 0; a < names.length; a += 1) {
    const ownA = owner(names[a]); if (!ownA) continue;
    for (let b = 0; b < names.length; b += 1) {
      if (a === b || owner(names[b]) === ownA || (owner(names[b]) && b < a)) continue;
      const distance = Number(mujoco.mj_geomDistance(model, data, a, b, .001, new Float64Array(6)));
      if (distance < -.0005) throw new RangeError(`Initial asset overlap ${names[a]} / ${names[b]} by ${(-distance * 1000).toFixed(2)} mm`);
    }
  }
}
async function load(modelPackage) {
  if (!modelPackage?.labBuilder || !modelPackage?.asset || !modelPackage?.baseSha256 || !modelPackage?.sha256) throw new Error('Lab Builder requires a registered labBuilder model package');
  const url = validatedModelUrl(modelPackage.asset), mj = await ensureMuJoCo(); dispose();
  let xml = await fetch(url, { cache: 'no-store' }).then(r => { if (!r.ok) throw new Error(`Model HTTP ${r.status}`); return r.text(); });
  if (await sha256Text(xml) !== modelPackage.baseSha256) throw new Error('Base OpenArm model checksum mismatch');
  xml = stripOpenArmReferenceWorkcellXml(xml);
  if (modelPackage.equipment?.length) { const normalized = validateOpenArmLabEquipment(modelPackage.equipment); equipment = appendOpenArmLabEquipmentXml(xml, normalized); xml = equipment.xml; }
  if (await sha256Text(xml) !== modelPackage.sha256) throw new Error('Compiled Lab Builder model checksum mismatch');
  model = mj.from_xml_string(xml); if (!model) throw new Error('MuJoCo failed to compile Lab Builder scene');
  data = new mj.MjData(model); descriptor = structuredClone(modelPackage); resolveAddresses(modelPackage.sha256); const initial = applyInitial(); validateClearance(); return initial;
}
function integrate() { servo.apply(); mujoco.mj_step(model, data); mujoco.mj_forward(model, data); if (!Array.from(data.qpos).every(Number.isFinite) || !Array.from(data.qvel).every(Number.isFinite)) throw new Error('Non-finite Lab Builder state'); }
function step(count = 1) { if (!Number.isInteger(count) || count < 1 || count > MAX_ADVANCE_STEPS_PER_REQUEST) throw new RangeError('Invalid step count'); if (!paused) for (let i = 0; i < count; i += 1) integrate(); return observation(); }
function stepSampled(count = 1, sampleEverySteps = 1) { if (!Number.isInteger(count) || count < 1 || count > MAX_ADVANCE_STEPS_PER_REQUEST) throw new RangeError('Invalid step count'); if (!Number.isInteger(sampleEverySteps) || sampleEverySteps < 1) throw new RangeError('Invalid sample cadence'); if (paused) return { observations: [observation()], executedSteps: 0, sampleEverySteps }; const expected = sampledObservationCount(count, sampleEverySteps); if (expected > MAX_SAMPLED_OBSERVATIONS_PER_ADVANCE) throw new RangeError('Observation sample budget exceeded'); const observations = []; for (let i = 1; i <= count; i += 1) { integrate(); if (i === count || i % sampleEverySteps === 0) observations.push(observation()); } return { observations, executedSteps: count, sampleEverySteps }; }
function validatedJointTarget(jointId, value) { const joint = jointState.get(jointId), actuator = positionActuator(jointId, true), target = Number(value); if (!joint || !Number.isFinite(target)) throw new Error(`Invalid joint target ${jointId}`); if (target < actuator.controlRange[0] || target > actuator.controlRange[1]) throw new RangeError(`${jointId} target outside actuator range`); if (target < joint.range[0] || target > joint.range[1]) throw new RangeError(`${jointId} target outside joint range`); return target; }

function validateToolWaypoints(payload = {}) {
  const side = payload.side; if (!['left', 'right'].includes(side)) throw new TypeError('Path validation side must be left or right');
  const waypoints = payload.waypoints; if (!Array.isArray(waypoints) || !waypoints.length || waypoints.length > PATH_LIMITS.waypoints) throw new RangeError(`Path validation requires 1..${PATH_LIMITS.waypoints} waypoints`);
  const minimumSeparationM = payload.minimumSeparationM == null ? .0015 : Number(payload.minimumSeparationM);
  const jointSampleStepRad = payload.jointSampleStepRad == null ? .035 : Number(payload.jointSampleStepRad);
  if (!Number.isFinite(minimumSeparationM) || minimumSeparationM < PATH_LIMITS.minSeparationM || minimumSeparationM > PATH_LIMITS.maxSeparationM) throw new RangeError('Invalid path separation margin');
  if (!Number.isFinite(jointSampleStepRad) || jointSampleStepRad < PATH_LIMITS.minJointStepRad || jointSampleStepRad > PATH_LIMITS.maxJointStepRad) throw new RangeError('Invalid joint path sampling step');
  const ignored = new Set(Array.isArray(payload.ignoreEquipmentIds) ? payload.ignoreEquipmentIds.map(String) : []); if (ignored.size > 8) throw new RangeError('Too many ignored equipment ids');
  const geomNames = Array.from({ length: Number(model.ngeom) }, (_, id) => nameForGeom(id));
  const ownerByGeom = new Map();
  for (const record of equipment?.records || []) for (const name of record.geometryIds || []) ownerByGeom.set(name, record.id);
  const robotGeoms = geomNames.map((name, id) => ({ name, id })).filter(item => typeof item.name === 'string' && item.name.includes(`_${side}_collision_`));
  const labGeoms = geomNames.map((name, id) => ({ name, id, owner: ownerByGeom.get(name) })).filter(item => item.owner && !ignored.has(item.owner));
  const scratch = new mujoco.MjData(model), fromto = new Float64Array(6); let totalSamples = 0, minimumObservedM = Infinity;
  try {
    scratch.qpos.set(data.qpos); scratch.qvel.set(data.qvel); mujoco.mj_forward(model, scratch);
    const armJoints = Array.from({ length: 7 }, (_, index) => jointState.get(`openarm_${side}_joint${index + 1}`));
    const diagnostics = [];
    for (let waypointIndex = 0; waypointIndex < waypoints.length; waypointIndex += 1) {
      const waypoint = waypoints[waypointIndex];
      if (!waypoint || !Array.isArray(waypoint.positionM) || waypoint.positionM.length !== 3 || waypoint.positionM.some(value => typeof value !== 'number' || !Number.isFinite(value))) throw new TypeError(`Waypoint ${waypointIndex + 1} requires finite positionM`);
      const solution = solveOpenArmIK(mujoco, model, scratch, jointState, { side, positionM: waypoint.positionM, ...(waypoint.quaternionWxyz ? { quaternionWxyz: waypoint.quaternionWxyz } : {}) });
      const targetValues = armJoints.map((joint, index) => validatedJointTarget(`openarm_${side}_joint${index + 1}`, solution.targetsRad[`openarm_${side}_joint${index + 1}`]));
      const startValues = armJoints.map(joint => Number(scratch.qpos[joint.qpos]));
      const maxDelta = Math.max(...targetValues.map((value, index) => Math.abs(value - startValues[index])));
      const samples = Math.max(1, Math.ceil(maxDelta / jointSampleStepRad)); totalSamples += samples; if (totalSamples > PATH_LIMITS.samples) throw new RangeError('Sampled path exceeds validation budget');
      let segmentMinimumM = Infinity;
      for (let sample = 1; sample <= samples; sample += 1) {
        const alpha = sample / samples;
        armJoints.forEach((joint, index) => { scratch.qpos[joint.qpos] = startValues[index] + (targetValues[index] - startValues[index]) * alpha; });
        mujoco.mj_forward(model, scratch);
        for (const robotGeom of robotGeoms) {
          for (const labGeom of labGeoms) {
            const distance = Number(mujoco.mj_geomDistance(model, scratch, robotGeom.id, labGeom.id, Math.max(.015, minimumSeparationM + .005), fromto));
            if (Number.isFinite(distance)) { segmentMinimumM = Math.min(segmentMinimumM, distance); minimumObservedM = Math.min(minimumObservedM, distance); }
            if (Number.isFinite(distance) && distance < minimumSeparationM) {
              return { valid: false, reason: 'robot_lab_clearance', side, waypointIndex, waypointLabel: String(waypoint.label || `waypoint ${waypointIndex + 1}`), sample, samples, minimumSeparationM, observedDistanceM: distance, robotGeom: robotGeom.name, labGeom: labGeom.name, equipmentId: labGeom.owner, totalSamples, activePlantMutated: false };
            }
          }
        }
      }
      diagnostics.push({ waypointIndex, label: String(waypoint.label || `waypoint ${waypointIndex + 1}`), samples, positionResidualM: solution.positionResidualM, orientationResidualRad: solution.orientationResidualRad, minimumRobotLabSeparationM: Number.isFinite(segmentMinimumM) ? segmentMinimumM : null });
    }
    return { valid: true, side, waypoints: diagnostics, minimumSeparationM, minimumObservedM: Number.isFinite(minimumObservedM) ? minimumObservedM : null, jointSampleStepRad, totalSamples, activePlantMutated: false, continuousCollisionGuarantee: false, scope: 'sampled narrow-phase selected-arm collision geometry versus all non-ignored authored lab geometry; task object is separately checked as a carried envelope' };
  } finally { scratch.delete(); }
}

function command(payload = {}) {
  let targets;
  if (payload.type === 'validate_tool_waypoints') {
    const validation = validateToolWaypoints(payload);
    motionPlan = { type: 'sampled_path_validation', validation, collisionFreePath: validation.valid, activePlantMutated: false };
    return observation();
  }
  if (payload.type === 'set_joint_targets' || payload.type === 'move_joint_targets') {
    if (!payload.targetsRad || typeof payload.targetsRad !== 'object' || Array.isArray(payload.targetsRad) || !Object.keys(payload.targetsRad).length) throw new Error('Joint targets required');
    targets = Object.fromEntries(Object.entries(payload.targetsRad).map(([id, value]) => [id, validatedJointTarget(id, value)]));
    const timing = servo.request(targets, payload.type === 'move_joint_targets' ? payload.durationSeconds : null);
    motionPlan = { type: payload.type, ...timing, collisionFreePath: false }; return observation();
  }
  if (payload.type === 'set_tool_target') {
    const solution = solveOpenArmIK(mujoco, model, data, jointState, payload);
    targets = Object.fromEntries(Object.entries(solution.targetsRad).map(([id, value]) => [id, validatedJointTarget(id, value)]));
    const timing = servo.request(targets, payload.durationSeconds ?? null);
    motionPlan = { ...solution, ...timing, collisionFreePath: false, plannerScope: 'IK endpoint; Lab Builder generated plans use a separate sampled narrow-phase validation before execution' }; return observation();
  }
  throw new Error(`Unsupported command ${payload.type}`);
}

self.onmessage = async event => {
  const { id, op, payload } = event.data || {};
  try {
    let result;
    if (op === 'load') result = await load(payload?.modelPackage);
    else if (op === 'reset') result = applyInitial();
    else if (op === 'step') result = step(payload?.count);
    else if (op === 'stepSampled') result = stepSampled(payload?.count, payload?.sampleEverySteps);
    else if (op === 'observe') result = observation();
    else if (op === 'command') result = command(payload);
    else if (op === 'pause') { paused = true; result = observation(); }
    else if (op === 'resume') { paused = false; result = observation(); }
    else if (op === 'dispose') { dispose(); result = true; }
    else throw new Error(`Unknown operation ${op}`);
    reply(id, true, result);
  } catch (error) { reply(id, false, null, String(error?.stack || error?.message || error)); }
};
