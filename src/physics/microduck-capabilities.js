import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord } from './model-registry.js';
import {
  MICRODUCK_COMPATIBILITY_SCHEMA_VERSION,
  MICRODUCK_DEPLOYED_POLICIES,
  MICRODUCK_POLICY_SHA256,
  MICRODUCK_RL_SOURCE,
  MICRODUCK_RUNTIME_SOURCE,
} from './microduck-source-audit.js';
import {
  MICRODUCK_CONTROL_INTERVAL_SECONDS,
  MICRODUCK_ACTION_WIDTH,
  MICRODUCK_OBSERVATION_WIDTH,
} from './microduck-controller.js';
import { MICRODUCK_GROUNDCONTACT_PACKAGE, MICRODUCK_KICK_PACKAGE, MICRODUCK_LOW_TRACTION_PACKAGE, MICRODUCK_WALK_PACKAGE } from './microduck-model-package.js';

// How a MicroDuck capability may be classified. Execution backend, capability and evidence
// stay separate: a workspace being physical does not make every skill in it physical.
export const MICRODUCK_CAPABILITY_STATUS = Object.freeze({
  PHYSICAL_VERIFIED: 'physical / verified',
  PHYSICAL_EXPERIMENTAL: 'physical / experimental',
  LEGACY: 'legacy',
  UNSUPPORTED: 'unsupported in physical mode',
});

const S = MICRODUCK_CAPABILITY_STATUS;

/**
 * Every capability the MicroDuck product exposes, classified individually.
 *
 * `physicalPolicy` is null for anything that may not run in physical mode. The physical
 * controller is constructed from exactly the non-null entries, so an unsupported capability
 * has no route into the physical session at all - it cannot fall through to legacy dynamics,
 * because the physical session has no legacy dynamics to fall through to.
 */
export const MICRODUCK_CAPABILITY_AUDIT = Object.freeze([
  Object.freeze({
    id: 'stand', label: 'Standing', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'stand',
    evidence: 'Native reference, 6 s: trunk holds 0.1161 m at 0.22 deg tilt with both soles in contact for 99.6% of physics steps, and 0.0024 m of drift.',
  }),
  Object.freeze({
    id: 'walk', label: 'Walking / velocity control', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'walking',
    evidence: 'Native reference, 6 s at vx 0.30 m/s: 0.694 m along the commanded axis (0.119 m/s) with 32/33 foot-contact transitions and a 0.45 air fraction per foot. '
      + 'Disabling actuation removes propulsion and the robot falls (tilt 67.8 deg); reducing sole/floor friction to 0.02 cuts commanded-axis distance by 77%.',
  }),
  Object.freeze({
    id: 'sit_stand', label: 'Sit / stand', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'sitstand',
    evidence: 'Native reference, 4 s with the posture flag set: the trunk descends from 0.1161 m to 0.062 m and holds there under contact; the rise returns it to standing.',
  }),
  Object.freeze({
    id: 'ground_pick', label: 'Ground pick', status: S.PHYSICAL_EXPERIMENTAL, physicalPolicy: 'ground_pick',
    evidence: 'Native reference, one 3 s phase cycle: a real crouch to 0.084 m and a return to 0.116 m, driven by contact. '
      + 'Classified experimental because the physical scene carries no object to pick up and the pinned sources define no pick-success criterion, so the motion is verified while the task is not.',
  }),
  Object.freeze({
    id: 'kick_left', label: 'Left-leg ball kick', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'kick_left',
    evidence: 'Native reference on the kick package: 7 named left-sole/ball contacts move the source 15 g ball 2.283 m. With the ball outside reach the same policy produces 0 contacts and exactly 0.000 m of ball motion.',
  }),
  Object.freeze({
    id: 'kick_right', label: 'Right-leg ball kick', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'kick_right',
    evidence: 'Native reference on the kick package: 5 named right-sole/ball contacts with a 2.66 N peak normal force move the source 15 g ball 2.514 m. The miss control produces 0 contacts and 0.000 m.',
  }),
  Object.freeze({
    id: 'recovery', label: 'Fall recovery', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'stand',
    evidence: 'Runs only on the source all-collision ground-contact plant. Positive and negative BAM/MuJoCo recovery controls are validated from declared setup orientations, actuator force, trunk pose and contact; reset is never counted as recovery.',
  }),
  Object.freeze({
    id: 'roulade', label: 'Roulade (forward roll)', status: S.PHYSICAL_EXPERIMENTAL, physicalPolicy: 'roulade',
    evidence: 'The deployed runtime includes roulade.onnx. It is routed to the broad source all-collision plant so trunk/head contact is represented by exact source meshes. The pinned microduck_rl revision does not contain the roulade task configuration, so this remains experimental rather than an exact task-training-plant claim.',
  }),
  Object.freeze({
    id: 'roller', label: 'Roller-mode locomotion', status: S.UNSUPPORTED, physicalPolicy: null,
    evidence: 'No matched physical roller plant exists at the pinned RL revision: it carries no roller model. The roller environment upstream is a different plant with wheel bodies and passive wheel hinges, '
      + 'and no revision was found that carries both the deployed robot model and a roller plant whose correspondence to these policy bytes could be shown. Running a roller policy on the walking model would be a policy/model mismatch.',
  }),
  Object.freeze({
    id: 'roller_crouch', label: 'Roller crouch', status: S.UNSUPPORTED, physicalPolicy: null,
    evidence: 'Same missing roller plant as roller-mode locomotion.',
  }),
]);

export const MICRODUCK_PHYSICAL_POLICIES = Object.freeze([...new Set(
  MICRODUCK_CAPABILITY_AUDIT.filter((item) => item.physicalPolicy).map((item) => item.physicalPolicy),
)]);

export const MICRODUCK_UNSUPPORTED_CAPABILITIES = Object.freeze(
  MICRODUCK_CAPABILITY_AUDIT.filter((item) => item.status === S.UNSUPPORTED).map((item) => item.id),
);

export function microduckCapability(id) {
  return MICRODUCK_CAPABILITY_AUDIT.find((item) => item.id === id) || null;
}

export function isPhysicallySupported(id) {
  const record = microduckCapability(id);
  return Boolean(record && record.physicalPolicy);
}

// Collision-plant routing is part of policy compatibility. A source policy is not allowed
// to run merely because its observation/action widths match: its task must also use the
// collision plant against which that behaviour is represented here.
export const MICRODUCK_CAPABILITY_PACKAGE_KEYS = Object.freeze({
  stand: Object.freeze(['groundContact']),
  walk: Object.freeze(['walk', 'lowTraction']),
  sit_stand: Object.freeze(['groundContact']),
  ground_pick: Object.freeze(['groundContact']),
  kick_left: Object.freeze(['kick']),
  kick_right: Object.freeze(['kick']),
  recovery: Object.freeze(['groundContact']),
  roulade: Object.freeze(['groundContact']),
});

export function microduckRequiredPackageKeys(capabilityId) {
  return MICRODUCK_CAPABILITY_PACKAGE_KEYS[capabilityId] || Object.freeze([]);
}

export function microduckPackageSupportsCapability(packageKey, capabilityId) {
  return microduckRequiredPackageKeys(capabilityId).includes(packageKey);
}

/**
 * The versioned compatibility identity a policy is validated against before it may run.
 * Everything that would silently change behaviour if it drifted is in here.
 */
export function microduckCompatibilityIdentity(modelPackage, policyId) {
  const policy = MICRODUCK_DEPLOYED_POLICIES.find((item) => item.id === policyId);
  if (!policy) throw new Error(`Unknown MicroDuck policy: ${policyId}`);
  return Object.freeze({
    schemaVersion: MICRODUCK_COMPATIBILITY_SCHEMA_VERSION,
    robotVariant: 'alpha',
    robotId: modelPackage.robotId,
    modelPackageId: modelPackage.id,
    modelSha256: modelPackage.sha256,
    collisionModelRevision: MICRODUCK_RL_SOURCE.revision,
    actuatorModelRevision: MICRODUCK_RL_SOURCE.revision,
    runtimeRevision: MICRODUCK_RUNTIME_SOURCE.revision,
    policyId,
    policyFile: policy.file,
    policySha256: MICRODUCK_POLICY_SHA256[policyId],
    observationSchema: `microduck-obs-${MICRODUCK_OBSERVATION_WIDTH}`,
    actionSchema: `microduck-act-${MICRODUCK_ACTION_WIDTH}`,
    controlIntervalSeconds: MICRODUCK_CONTROL_INTERVAL_SECONDS,
  });
}

const COMPATIBILITY_FIELDS = Object.freeze([
  'schemaVersion', 'robotVariant', 'robotId', 'modelPackageId', 'modelSha256',
  'collisionModelRevision', 'actuatorModelRevision', 'runtimeRevision',
  'policyId', 'policyFile', 'policySha256', 'observationSchema', 'actionSchema', 'controlIntervalSeconds',
]);

/**
 * Validate a policy against a model package before enabling it.
 *
 * Returns `{ ok: true, identity }` or throws. A mismatch is a refusal, never a quiet
 * downgrade to "lower fidelity".
 */
export function assertMicroDuckCompatibility(modelPackage, policyId, presented = null) {
  const capability = MICRODUCK_CAPABILITY_AUDIT.find((item) => item.physicalPolicy === policyId);
  if (!capability) {
    const unsupported = MICRODUCK_CAPABILITY_AUDIT.find((item) => item.id === policyId || item.physicalPolicy === policyId);
    const reason = unsupported ? unsupported.evidence : 'it is not part of the physical capability set';
    throw new Error(`MicroDuck policy ${policyId} is unsupported in physical mode: ${reason}`);
  }
  const identity = microduckCompatibilityIdentity(modelPackage, policyId);
  if (presented) {
    for (const field of COMPATIBILITY_FIELDS) {
      if (presented[field] === undefined) continue;
      if (presented[field] !== identity[field]) {
        throw new Error(`MicroDuck compatibility mismatch on ${field}: presented ${presented[field]}, package requires ${identity[field]}`);
      }
    }
    for (const field of Object.keys(presented)) {
      if (!COMPATIBILITY_FIELDS.includes(field)) throw new Error(`Unknown MicroDuck compatibility field: ${field}`);
    }
  }
  return { ok: true, identity, capability };
}

const PHYSICAL_LIMITATIONS = Object.freeze([
  'Task-specific collision geometry is source-derived, not a single universal approximation: locomotion uses the pinned robot_walk.xml reduced set; explicit body-on-ground skills use robot_allcollisions.xml; kick uses robot_allcollisions.xml plus the source ball.xml prop.',
  'The committed STL collider bytes are copied exactly from the pinned microduck_rl revision and retain upstream Creative Commons BY-SA-NC terms; upstream does not state a CC version. RoboBuddy original software remains MIT-scoped separately.',
  'The interactive physical workspace uses BAM 1.0.1 XL330/M6 dynamics with the deployed firmware-gain schedule and deployed median-of-three IMU preprocessing. Training-only domain randomisation/delay is retained as a separate source reference profile rather than silently mixed into deployment rehearsal.',
  'Roller-mode locomotion and roller crouch are unsupported in physical mode: no matched roller plant exists at the pinned revision. They are not routed anywhere else.',
  'Native/browser agreement and source-model fidelity are software/model evidence, not assembled-hardware calibration. Battery/internal resistance, bus latency, thermal effects, individual servo variation, wear and real contact materials remain hardware-validation items.',
]);

export const MICRODUCK_PHYSICAL_CAPABILITY = capabilityRecord({
  backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
  capability: 'single-authority MicroDuck alpha free-base biped workspace with the matched deployed 61-value controller, contact-driven locomotion, contact-driven kicking, physical fall recovery, live async Python and causal task evaluation',
  evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
  limitations: [...PHYSICAL_LIMITATIONS],
});

export const MICRODUCK_LEGACY_CAPABILITY = capabilityRecord({
  backend: EXECUTION_BACKENDS.LEGACY,
  capability: 'legacy reference-aligned policy demonstrator',
  evidence: MODEL_EVIDENCE.MODEL_DERIVED,
  limitations: [
    'Approximate browser dynamics with a bounded root-motion surrogate. Locomotion, kicks, boundaries and recovery in this workspace are configured browser rules, not physics.',
    'Its ONNX evidence proves deterministic CPU/browser inference parity for a fixed 1x61 input only. It establishes no RL-environment, locomotion, contact, MuJoCo-model or hardware parity.',
    'It is never used as a fallback for the physical workspace; selecting it is an explicit choice.',
  ],
});

export const MICRODUCK_PHYSICAL_PACKAGES = Object.freeze({
  walk: MICRODUCK_WALK_PACKAGE,
  lowTraction: MICRODUCK_LOW_TRACTION_PACKAGE,
  groundContact: MICRODUCK_GROUNDCONTACT_PACKAGE,
  kick: MICRODUCK_KICK_PACKAGE,
});
