import { graspState, usableContact } from './openarm-observation.js';

export const OPENARM_LAB_OBSERVATION_PROFILE_VERSION = 'robobuddy.openarm.observation-profile.v1';
export const OPENARM_LAB_OBSERVATION_PROFILES = Object.freeze({
  simulator_ground_truth: Object.freeze({
    schema_version: OPENARM_LAB_OBSERVATION_PROFILE_VERSION,
    id: 'simulator_ground_truth',
    label: 'Simulator ground truth (development only)',
    source: 'authoritative MuJoCo PhysicsSession',
    sample_period_s: .002,
    latency_s: 0,
    position_noise_std_m: 0,
    joint_noise_std_rad: 0,
    orientation_noise_std_rad: 0,
    contact_state: 'full simulator contact list/forces available to development tools and hidden evaluator',
    assumed_hardware_interface: false,
    camera_perception: false,
    hardware_validated: false,
  }),
  synthetic_estimator_v1: Object.freeze({
    schema_version: OPENARM_LAB_OBSERVATION_PROFILE_VERSION,
    id: 'synthetic_estimator_v1',
    label: 'Synthetic delayed/noisy estimator sensitivity profile',
    source: 'deterministic perturbation of delayed MuJoCo observations; not camera inference',
    sample_period_s: .02,
    latency_s: .04,
    position_noise_std_m: .002,
    joint_noise_std_rad: .004,
    orientation_noise_std_rad: .012,
    contact_state: 'delayed binary gripper/support state only; no contact force, depth, full pair list or tactile-sensor claim',
    assumed_hardware_interface: true,
    camera_perception: false,
    hardware_validated: false,
  }),
});

function xorshift32(seed) {
  let x = seed >>> 0 || 0x6d2b79f5;
  return () => { x ^= x << 13; x ^= x >>> 17; x ^= x << 5; return (x >>> 0) / 0x100000000; };
}
function gaussian(random) {
  const u = Math.max(1e-12, random()), v = Math.max(1e-12, random());
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
function noisyQuaternion(q, sigma, random) {
  const axis = [gaussian(random), gaussian(random), gaussian(random)];
  const n = Math.hypot(...axis) || 1, angle = gaussian(random) * sigma;
  const s = Math.sin(angle / 2) / n, d = [Math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s];
  const [w,x,y,z] = d, [a,b,c,e] = q;
  const out = [w*a-x*b-y*c-z*e, w*b+x*a+y*e-z*c, w*c-x*e+y*a+z*b, w*e+x*c-y*b+z*a];
  const m = Math.hypot(...out) || 1; return out.map(value => value / m);
}
function contactEstimates(source) {
  const equipment = source.openarm?.equipment || [];
  const byGeom = new Map();
  for (const record of equipment) for (const geom of record.geometryIds || []) byGeom.set(geom, record.bodyId);
  const supportPairs = [];
  for (const contact of source.contacts || []) {
    if (!usableContact(contact)) continue;
    const body1 = byGeom.get(contact.geom1Name), body2 = byGeom.get(contact.geom2Name);
    if (body1 && !body2) supportPairs.push({ objectId: body1, supportGeom: contact.geom2Name });
    if (body2 && !body1) supportPairs.push({ objectId: body2, supportGeom: contact.geom1Name });
    if (body1 && body2) {
      supportPairs.push({ objectId: body1, supportGeom: contact.geom2Name });
      supportPairs.push({ objectId: body2, supportGeom: contact.geom1Name });
    }
  }
  const grasp = {};
  for (const record of equipment) {
    grasp[record.bodyId] = {};
    for (const side of ['left','right']) {
      const state = graspState(source, side, record.bodyId);
      grasp[record.bodyId][side] = { bilateralContact: Boolean(state.bilateralContact), anyGripperContact: Boolean(state.anyGripperContact) };
    }
  }
  return { grasp, supportPairs, source: 'delayed simulator-derived binary state for sensitivity testing; no force/tactile hardware claim' };
}

export class OpenArmSyntheticEstimator {
  constructor({ seed = 1 } = {}) { this.reset(seed); }
  reset(seed = 1) { this.seed = seed >>> 0; this.random = xorshift32(this.seed); this.queue = []; this.lastSampleTimeS = -Infinity; this.sequence = 0; }
  push(source) {
    const time = Number(source?.simulationTimeSeconds); if (!Number.isFinite(time)) return;
    const profile = OPENARM_LAB_OBSERVATION_PROFILES.synthetic_estimator_v1;
    if (time - this.lastSampleTimeS + 1e-12 < profile.sample_period_s) return;
    this.lastSampleTimeS = time; this.queue.push(structuredClone(source));
    while (this.queue.length > 128) this.queue.shift();
  }
  observe(currentSimulationTimeS) {
    const profile = OPENARM_LAB_OBSERVATION_PROFILES.synthetic_estimator_v1, target = currentSimulationTimeS - profile.latency_s;
    let source = null;
    for (const candidate of this.queue) if (candidate.simulationTimeSeconds <= target + 1e-12) source = candidate; else break;
    if (!source) return { valid:false, reason:'latency_buffer_not_ready', profileId:profile.id, sourceSimulationTimeS:null, deliveredSimulationTimeS:currentSimulationTimeS, seed:this.seed, hardwareValidated:false, cameraPerception:false };
    const joints = Object.fromEntries(Object.entries(source.joints || {}).map(([id,joint]) => [id, {
      positionRad: Number(joint.positionRad) + gaussian(this.random) * profile.joint_noise_std_rad,
      velocityRadS: Number(joint.velocityRadS || 0),
    }]));
    const bodies = Object.fromEntries(Object.entries(source.bodies || {}).map(([id,body]) => [id, {
      frame: body.frame || 'mujoco_world',
      positionM: body.positionM.map(value => Number(value) + gaussian(this.random) * profile.position_noise_std_m),
      quaternionWxyz: noisyQuaternion(body.quaternionWxyz, profile.orientation_noise_std_rad, this.random),
      ...(Array.isArray(body.linearVelocityMS) ? {linearVelocityMS:[...body.linearVelocityMS]} : {}),
      ...(Array.isArray(body.angularVelocityRadS) ? {angularVelocityRadS:[...body.angularVelocityRadS]} : {}),
    }]));
    const pinchReferences = Object.fromEntries(Object.entries(source.openarm?.pinchReferences || {}).map(([side,record]) => [side, {
      frame: record.frame || 'mujoco_world',
      positionM: record.positionM.map(value => Number(value) + gaussian(this.random) * profile.position_noise_std_m),
      quaternionWxyz: noisyQuaternion(record.quaternionWxyz, profile.orientation_noise_std_rad, this.random),
    }]));
    return {
      valid:true, sequence:++this.sequence, profileId:profile.id, sourceSimulationTimeS:source.simulationTimeSeconds,
      deliveredSimulationTimeS:currentSimulationTimeS, ageSeconds:currentSimulationTimeS-source.simulationTimeSeconds,
      seed:this.seed, joints, bodies, pinchReferences, contactEstimates:contactEstimates(source), contactsAvailable:false,
      contactForcesAvailable:false, source:'synthetic delayed/noisy estimator generated from simulator state for pre-hardware sensitivity only',
      hardwareValidated:false, cameraPerception:false,
    };
  }
}

export function syntheticEstimatorAsProgramObservation(sensor) {
  if (!sensor?.valid) return null;
  return {
    simulationTimeSeconds: sensor.deliveredSimulationTimeS,
    contactsReadable: false,
    contactCount: 0,
    contacts: [],
    joints: structuredClone(sensor.joints || {}),
    bodies: structuredClone(sensor.bodies || {}),
    openarm: {
      observationContract: sensor.profileId,
      pinchReferences: structuredClone(sensor.pinchReferences || {}),
      contactEstimates: structuredClone(sensor.contactEstimates || { grasp:{}, supportPairs:[] }),
      sourceSimulationTimeS: sensor.sourceSimulationTimeS,
      observationAgeSeconds: sensor.ageSeconds,
    },
  };
}
