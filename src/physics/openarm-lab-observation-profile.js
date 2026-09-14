export const OPENARM_LAB_OBSERVATION_PROFILES = Object.freeze({
  simulator_ground_truth: Object.freeze({
    id: 'simulator_ground_truth',
    label: 'Simulator ground truth (development)',
    source: 'MuJoCo authoritative state',
    hardwareAlignment: 'none',
    samplingPeriodS: 0.001,
    latencyS: 0,
    restrictions: Object.freeze([]),
  }),
  synthetic_estimator_v1: Object.freeze({
    id: 'synthetic_estimator_v1',
    label: 'Synthetic estimator sensitivity profile',
    source: 'Delayed/noisy transform and joint estimates generated from simulation for software sensitivity testing',
    hardwareAlignment: 'assumed interface; parameters are illustrative estimates, not measurements and not camera perception',
    samplingPeriodS: 0.02,
    latencyS: 0.04,
    jointPositionStdRad: 0.002,
    jointVelocityStdRadS: 0.01,
    bodyPositionStdM: 0.003,
    bodyOrientationStdRad: 0.008,
    restrictions: Object.freeze([
      'No exact contact forces or MuJoCo contact list are exposed through this profile.',
      'Object and tool poses are delayed/noisy estimates; evaluator ground truth remains private to application evaluation.',
      'This is not rendered-camera perception, calibration, or a hardware sensor replica.',
    ]),
  }),
});

function mulberry32(seed) {
  let state = seed >>> 0;
  return () => {
    state |= 0;
    state = state + 0x6D2B79F5 | 0;
    let t = Math.imul(state ^ state >>> 15, 1 | state);
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}
function normal(rng) {
  const u1 = Math.max(Number.EPSILON, rng());
  const u2 = rng();
  return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}
function normalizeQuaternion(q) {
  const n = Math.hypot(...q);
  return n > 0 ? q.map(v => v / n) : [1, 0, 0, 0];
}
function multiplyQuaternion(a, b) {
  const [aw, ax, ay, az] = a, [bw, bx, by, bz] = b;
  return [
    aw*bw-ax*bx-ay*by-az*bz,
    aw*bx+ax*bw+ay*bz-az*by,
    aw*by-ax*bz+ay*bw+az*bx,
    aw*bz+ax*by-ay*bx+az*bw,
  ];
}
function perturbQuaternion(q, std, rng) {
  const rx = normal(rng) * std, ry = normal(rng) * std, rz = normal(rng) * std;
  const angle = Math.hypot(rx, ry, rz);
  if (angle < 1e-12) return [...q];
  const s = Math.sin(angle / 2) / angle;
  return normalizeQuaternion(multiplyQuaternion(q, [Math.cos(angle / 2), rx*s, ry*s, rz*s]));
}

export class OpenArmSyntheticEstimator {
  constructor({ seed = 1, profileId = 'synthetic_estimator_v1' } = {}) {
    if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) throw new RangeError('Estimator seed must be a uint32');
    const profile = OPENARM_LAB_OBSERVATION_PROFILES[profileId];
    if (!profile || profileId === 'simulator_ground_truth') throw new TypeError('Synthetic estimator requires a synthetic profile');
    this.profile = profile;
    this.seed = seed >>> 0;
    this.rng = mulberry32(this.seed);
    this.queue = [];
    this.lastSampleTime = -Infinity;
    this.sequence = 0;
  }
  reset(seed = this.seed) {
    if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) throw new RangeError('Estimator seed must be a uint32');
    this.seed = seed >>> 0;
    this.rng = mulberry32(this.seed);
    this.queue.length = 0;
    this.lastSampleTime = -Infinity;
    this.sequence = 0;
  }
  push(observation) {
    const time = Number(observation?.simulationTimeSeconds);
    if (!Number.isFinite(time)) return;
    if (time + 1e-12 < this.lastSampleTime + this.profile.samplingPeriodS) return;
    this.lastSampleTime = time;
    this.queue.push(structuredClone(observation));
    while (this.queue.length > 64) this.queue.shift();
  }
  observe(currentSimulationTimeS) {
    const threshold = Number(currentSimulationTimeS) - this.profile.latencyS;
    if (!Number.isFinite(threshold)) return this.invalid('invalid_current_time');
    let source = null;
    while (this.queue.length && Number(this.queue[0].simulationTimeSeconds) <= threshold + 1e-12) source = this.queue.shift();
    if (!source) return this.invalid('latency_buffer_not_ready');
    const joints = {};
    for (const [id, joint] of Object.entries(source.joints || {})) {
      joints[id] = {
        positionRad: Number(joint.positionRad) + normal(this.rng) * this.profile.jointPositionStdRad,
        velocityRadS: Number(joint.velocityRadS) + normal(this.rng) * this.profile.jointVelocityStdRadS,
      };
    }
    const bodies = {};
    for (const [id, body] of Object.entries(source.bodies || {})) {
      if (!Array.isArray(body.positionM) || !Array.isArray(body.quaternionWxyz)) continue;
      bodies[id] = {
        frame: body.frame || 'mujoco_world',
        positionM: body.positionM.map(v => Number(v) + normal(this.rng) * this.profile.bodyPositionStdM),
        quaternionWxyz: perturbQuaternion(body.quaternionWxyz.map(Number), this.profile.bodyOrientationStdRad, this.rng),
      };
    }
    const pinchReferences = {};
    for (const [side, pose] of Object.entries(source.openarm?.pinchReferences || {})) {
      pinchReferences[side] = {
        frame: pose.frame || 'mujoco_world',
        positionM: pose.positionM.map(v => Number(v) + normal(this.rng) * this.profile.bodyPositionStdM),
        quaternionWxyz: perturbQuaternion(pose.quaternionWxyz.map(Number), this.profile.bodyOrientationStdRad, this.rng),
      };
    }
    return {
      valid: true,
      profileId: this.profile.id,
      seed: this.seed,
      sampleSequence: ++this.sequence,
      sourceSimulationTimeSeconds: Number(source.simulationTimeSeconds),
      deliveredSimulationTimeSeconds: Number(currentSimulationTimeS),
      nominalLatencySeconds: this.profile.latencyS,
      samplingPeriodSeconds: this.profile.samplingPeriodS,
      joints,
      bodies,
      pinchReferences,
      contactsAvailable: false,
      limitations: [...this.profile.restrictions],
      hardwareValidated: false,
      cameraPerception: false,
    };
  }
  invalid(reason) {
    return {
      valid: false,
      reason,
      profileId: this.profile.id,
      seed: this.seed,
      contactsAvailable: false,
      hardwareValidated: false,
      cameraPerception: false,
    };
  }
}
