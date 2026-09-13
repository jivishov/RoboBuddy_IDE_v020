import { ASIMOV_SOURCE } from '../physics/asimov-generated.js';

export const ASIMOV_WHOLE_BODY_REVISION = 'robobuddy.asimov.whole-body.v1';
export const ASIMOV_WHOLE_BODY_LIMITS = Object.freeze({
  maxKeyframes: 32,
  maxDurationSeconds: 12,
  maxKeyframeSeconds: 2,
  controlIntervalSeconds: 0.05,
  minPelvisHeightM: 0.42,
  maxTiltRad: 0.70,
});

const RANGES = Object.freeze(Object.fromEntries(ASIMOV_SOURCE.joints.map((joint) => [joint.id, joint.rangeRad])));
const VELOCITY_LIMITS = Object.freeze(Object.fromEntries(ASIMOV_SOURCE.joints.map((joint) => [joint.id, joint.velocityLimitRadS])));
const MOTION_INTENTS = Object.freeze(['step', 'walk', 'turn', 'squat', 'reach', 'gesture', 'custom']);
const SUPPORT_HINTS = Object.freeze(['double', 'left', 'right', 'any']);

const fail = (message) => { throw new RangeError(message); };
function plain(value, allowed, label) {
  if (!value || typeof value !== 'object' || Array.isArray(value)
      || (Object.getPrototypeOf(value) !== Object.prototype && Object.getPrototypeOf(value) !== null)) fail(`${label} must be a plain object`);
  for (const key of Object.keys(value)) if (!allowed.includes(key)) fail(`Unexpected ${label} field: ${key}`);
}
function finite(value, minimum, maximum, label) {
  if (!Number.isFinite(value) || value < minimum || value > maximum) fail(`${label} must be ${minimum}..${maximum}`);
  return Number(value);
}
function targets(value) {
  plain(value, Object.keys(RANGES), 'targets_rad');
  if (!Object.keys(value).length) fail('targets_rad must not be empty');
  const result = {};
  for (const [jointId, raw] of Object.entries(value)) result[jointId] = finite(raw, RANGES[jointId][0], RANGES[jointId][1], jointId);
  return result;
}

export function asimovWholeBodySchema(version, targetSchema) {
  return {
    type: 'object',
    properties: {
      schema_version: version,
      command: { type: 'string', const: 'run_whole_body_motion' },
      motion_intent: { type: 'string', enum: [...MOTION_INTENTS], description: 'Descriptive intent only. Physics, not the label, decides whether the motion succeeds.' },
      abort_on_instability: { type: 'boolean', description: 'Abort on a fall-like state, sensor fault, or non-foot ground contact. Default true.' },
      keyframes: {
        type: 'array', minItems: 1, maxItems: ASIMOV_WHOLE_BODY_LIMITS.maxKeyframes,
        items: {
          type: 'object',
          properties: {
            duration_seconds: { type: 'number', exclusiveMinimum: 0, maximum: ASIMOV_WHOLE_BODY_LIMITS.maxKeyframeSeconds },
            targets_rad: targetSchema,
            phase: { type: 'string', minLength: 1, maxLength: 48 },
            support: { type: 'string', enum: [...SUPPORT_HINTS], description: 'Optional expected support at the end of this keyframe; recorded, never fabricated.' },
          },
          required: ['duration_seconds', 'targets_rad'],
          additionalProperties: false,
        },
      },
    },
    required: ['schema_version', 'command', 'keyframes'],
    additionalProperties: false,
  };
}

export function parseAsimovWholeBody(input) {
  plain(input, ['schema_version', 'command', 'motion_intent', 'abort_on_instability', 'keyframes'], 'whole-body motion');
  if (input.command !== 'run_whole_body_motion') fail('Unknown whole-body command');
  const motionIntent = input.motion_intent === undefined ? 'custom' : String(input.motion_intent);
  if (!MOTION_INTENTS.includes(motionIntent)) fail(`motion_intent must be one of ${MOTION_INTENTS.join(', ')}`);
  if (input.abort_on_instability !== undefined && typeof input.abort_on_instability !== 'boolean') fail('abort_on_instability must be boolean');
  if (!Array.isArray(input.keyframes) || !input.keyframes.length || input.keyframes.length > ASIMOV_WHOLE_BODY_LIMITS.maxKeyframes) {
    fail(`Provide 1..${ASIMOV_WHOLE_BODY_LIMITS.maxKeyframes} whole-body keyframes`);
  }
  let totalSeconds = 0;
  const keyframes = input.keyframes.map((frame, index) => {
    plain(frame, ['duration_seconds', 'targets_rad', 'phase', 'support'], `keyframe ${index + 1}`);
    const durationSeconds = finite(frame.duration_seconds, Number.MIN_VALUE, ASIMOV_WHOLE_BODY_LIMITS.maxKeyframeSeconds, `keyframe ${index + 1} duration_seconds`);
    totalSeconds += durationSeconds;
    const phase = frame.phase === undefined ? `keyframe-${index + 1}` : String(frame.phase);
    if (!phase.length || phase.length > 48) fail(`keyframe ${index + 1} phase must be 1..48 characters`);
    const support = frame.support === undefined ? 'any' : String(frame.support);
    if (!SUPPORT_HINTS.includes(support)) fail(`keyframe ${index + 1} support must be ${SUPPORT_HINTS.join(', ')}`);
    return Object.freeze({ durationSeconds, targetsRad: targets(frame.targets_rad), phase, support });
  });
  if (totalSeconds > ASIMOV_WHOLE_BODY_LIMITS.maxDurationSeconds + 1e-10) fail(`Whole-body motion exceeds ${ASIMOV_WHOLE_BODY_LIMITS.maxDurationSeconds} simulation seconds`);
  return Object.freeze({
    command: 'run_whole_body_motion', motionIntent,
    abortOnInstability: input.abort_on_instability !== false,
    keyframes: Object.freeze(keyframes), totalSeconds,
  });
}

function align(seconds, dt, label) {
  const steps = Math.round(seconds / dt);
  if (steps < 1 || Math.abs(steps * dt - seconds) > 1e-9) fail(`${label} must align with the selected physics timestep`);
  return steps;
}
function supportSnapshot(sim) {
  const contacts = sim.getContacts?.() || {};
  const left = Array.isArray(contacts.leftFootFloor) && contacts.leftFootFloor.length > 0;
  const right = Array.isArray(contacts.rightFootFloor) && contacts.rightFootFloor.length > 0;
  return {
    left, right,
    mode: left && right ? 'double' : left ? 'left' : right ? 'right' : 'none',
    nonFootGroundContacts: Array.isArray(contacts.otherBodyFloor) ? contacts.otherBodyFloor.length : 0,
  };
}
function yawOf(quaternionWxyz) {
  const [w, x, y, z] = quaternionWxyz || [1, 0, 0, 0];
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}
function instability(state, support) {
  const root = state?.root;
  if (!root || root.mode !== 'free-base') return 'whole-body motion requires a free-base Asimov scene';
  if (!state.actuation_enabled) return 'actuation-disabled';
  if (!Number.isFinite(root.position_m?.[2]) || root.position_m[2] < ASIMOV_WHOLE_BODY_LIMITS.minPelvisHeightM) return 'low-pelvis';
  if (!Number.isFinite(root.tilt_rad) || root.tilt_rad > ASIMOV_WHOLE_BODY_LIMITS.maxTiltRad) return 'excess-tilt';
  if (support.nonFootGroundContacts > 0) return 'non-foot-ground-contact';
  const sensor = state.standing_assessment?.sensorFeedback;
  if (sensor?.status === 'fault') return `sensor-fault:${sensor.fault?.reason || 'unknown'}`;
  return null;
}
function interpolate(from, to, alpha) {
  const result = {};
  for (const jointId of Object.keys(RANGES)) result[jointId] = from[jointId] + (to[jointId] - from[jointId]) * alpha;
  return result;
}
function fullMeasuredState(state) {
  const result = {};
  for (const jointId of Object.keys(RANGES)) {
    const value = state?.joints?.[jointId]?.position_rad;
    if (!Number.isFinite(value)) fail(`Measured joint position unavailable for ${jointId}`);
    result[jointId] = value;
  }
  return result;
}
function compileTargets(program, start) {
  let previous = { ...start };
  const compiled = [];
  for (const [index, frame] of program.keyframes.entries()) {
    const next = { ...previous, ...frame.targetsRad };
    for (const jointId of Object.keys(RANGES)) {
      const rate = Math.abs(next[jointId] - previous[jointId]) / frame.durationSeconds;
      if (rate > VELOCITY_LIMITS[jointId] + 1e-9) fail(`keyframe ${index + 1} requests ${jointId} target motion faster than its source velocity limit`);
    }
    compiled.push({ ...frame, from: previous, to: next });
    previous = next;
  }
  return compiled;
}

/**
 * Execute a finite agent-generated full-body target trajectory against the existing MuJoCo plant.
 * The worker keeps its declared balance feedback active, but no root pose/velocity/force is written.
 * Completion means the requested trajectory executed; locomotion success is reported from measured
 * root/contact state rather than inferred from the motion_intent label.
 */
export async function executeAsimovWholeBodyMotion(sim, program, { dt, guard, advance }) {
  if (!Number.isFinite(dt) || dt <= 0) fail('Unknown scene timestep');
  const controlSeconds = ASIMOV_WHOLE_BODY_LIMITS.controlIntervalSeconds;
  align(controlSeconds, dt, 'whole-body control interval');
  for (const frame of program.keyframes) align(frame.durationSeconds, dt, 'whole-body keyframe duration');
  guard();
  const before = sim.getState();
  if (!before?.root || before.root.mode !== 'free-base') fail('Whole-body motion requires a free-base Asimov scene');
  const compiled = compileTargets(program, fullMeasuredState(before));
  const origin = [...before.root.position_m];
  const initialYaw = yawOf(before.root.quaternion_wxyz);
  let maxTiltRad = Number(before.root.tilt_rad || 0);
  let minPelvisHeightM = Number(before.root.position_m[2]);
  let supportTransitions = 0;
  let previousSupport = supportSnapshot(sim).mode;
  let executedSeconds = 0;
  const samples = [];

  await sim.engageWholeBody({ assertActive: guard });
  guard();
  for (const [index, frame] of compiled.entries()) {
    const frameSteps = align(frame.durationSeconds, dt, `keyframe ${index + 1} duration`);
    const controlSteps = align(controlSeconds, dt, 'whole-body control interval');
    let physicsSteps = 0;
    while (physicsSteps < frameSteps) {
      guard();
      const nextSteps = Math.min(controlSteps, frameSteps - physicsSteps);
      const alpha = (physicsSteps + nextSteps) / frameSteps;
      await sim.applyWholeBodyTargets(interpolate(frame.from, frame.to, alpha), {
        maxSteps: Math.max(frameSteps, 1), assertActive: guard,
      });
      guard();
      const seconds = nextSteps * dt;
      await advance(seconds);
      physicsSteps += nextSteps;
      executedSeconds += seconds;
      guard();
      const state = sim.getState();
      const support = supportSnapshot(sim);
      if (support.mode !== previousSupport) { supportTransitions += 1; previousSupport = support.mode; }
      maxTiltRad = Math.max(maxTiltRad, Number(state.root?.tilt_rad || 0));
      minPelvisHeightM = Math.min(minPelvisHeightM, Number(state.root?.position_m?.[2] ?? Infinity));
      const reason = instability(state, support);
      if (program.abortOnInstability && reason) {
        return {
          programRevision: ASIMOV_WHOLE_BODY_REVISION,
          executionStatus: 'aborted-instability',
          motionIntent: program.motionIntent,
          completedKeyframes: index,
          advancedSeconds: executedSeconds,
          instabilityReason: reason,
          rootDisplacementM: state.root.position_m.map((value, axis) => value - origin[axis]),
          horizontalDisplacementM: Math.hypot(state.root.position_m[0] - origin[0], state.root.position_m[1] - origin[1]),
          yawChangeRad: yawOf(state.root.quaternion_wxyz) - initialYaw,
          maxTiltRad, minPelvisHeightM, supportTransitions,
          note: 'The physical plant became unstable. No root correction, teleport, hidden reset or success event was applied.',
        };
      }
    }
    const state = sim.getState();
    const support = supportSnapshot(sim);
    samples.push({
      keyframe: index + 1, phase: frame.phase, expectedSupport: frame.support, observedSupport: support.mode,
      supportMatched: frame.support === 'any' || frame.support === support.mode,
      simulationTimeSeconds: state.simulation_time_s,
      rootPositionM: [...state.root.position_m], tiltRad: state.root.tilt_rad,
    });
  }

  const after = sim.getState();
  const dx = after.root.position_m[0] - origin[0];
  const dy = after.root.position_m[1] - origin[1];
  const dz = after.root.position_m[2] - origin[2];
  return {
    programRevision: ASIMOV_WHOLE_BODY_REVISION,
    executionStatus: 'completed',
    motionIntent: program.motionIntent,
    completedKeyframes: compiled.length,
    advancedSeconds: executedSeconds,
    rootDisplacementM: [dx, dy, dz],
    horizontalDisplacementM: Math.hypot(dx, dy),
    yawChangeRad: yawOf(after.root.quaternion_wxyz) - initialYaw,
    maxTiltRad, minPelvisHeightM, supportTransitions, samples,
    note: 'All keyframes executed through bounded joint actuation and contact physics. Completion is not proof that a requested step/walk/turn was achieved; use measured displacement, support transitions and final state. This is simulator-only and not hardware calibration.',
  };
}
