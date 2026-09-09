import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord } from './model-registry.js';

const LEGACY_LIMITS = Object.freeze([
  'Physical MuJoCo backend not yet active for this workspace.',
  'Task outcome may still depend on legacy modeled behavior.',
  'No hardware-validation claim is made.',
]);

export const PHYSICS_PREVIEW_CAPABILITIES = Object.freeze({
  so101: capabilityRecord({
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'single-authority SO-101 rigid-body block-transfer workspace with live async Python and physical task evaluation',
    evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
    limitations: [
      'The supported task is a synthetic rigid-body benchmark, not a hardware-calibrated laboratory workflow.',
      'SO-101 model provenance is source-derived from the pinned MuJoCo Menagerie adaptation; servo/controller parameters are simulation estimates and hardware alignment remains calibration-required.',
      'No liquid, meniscus, vacuum, suction, tactile-sensor, force-sensor, or physical-hardware control capability is claimed.',
    ],
  }),
  openarm: capabilityRecord({
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'single-authority OpenArm V2 bimanual dry-stack workspace with shared free-body objects, live async Python and causal task evaluation',
    evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
    limitations: [
      'Kinematics, mirrored V2 joint frames, link inertias and source simulation actuator semantics are pinned to enactic/openarm_mujoco V2; browser collision meshes are explicit primitive surrogates rather than upstream mesh collisions.',
      'The hotplate, ring stand/gauze, staging supports, flask and beaker are dry benchmark geometry rather than measured laboratory hardware.',
      'No thermal, liquid, tactile/force-sensor, CAN timing or hardware-safety validation is claimed.',
    ],
  }),
  lekiwi: capabilityRecord({
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'single-authority LeKiwi V1 flat-floor holonomic mobile-manipulation workspace with explicit passive omni-wheel rollers, a free payload, live async Python and causal task evaluation',
    evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
    limitations: [
      'Wheel placement, wheel axes, wheel radius, roller layout, chassis inertials and the arm mount are source-derived from pinned SIGRobotics-UIUC/LeKiwi geometry; the URDF uses CAD default densities, so the resulting base mass is a CAD figure rather than a weighed robot.',
      'Validated only on a declared flat indoor floor. Carpet, thresholds, curbs, rough terrain, suspension, high-speed carrying, wheel wear, battery droop, motor thermal behaviour, real odometry and SLAM are outside the evidence scope.',
      'The transfer bench, receiving zone and beaker are configured educational geometry from the pinned legacy courier scenario; beaker mass, friction and gripping force are estimates, not measurements.',
      'No hardware comparison exists. Nothing here establishes installed-LeKiwi calibration, payload rating, real grip force, or hardware safety.',
    ],
  }),
  unitree: capabilityRecord({ backend: EXECUTION_BACKENDS.LEGACY, capability: 'kinematic joint-pose inspection only', evidence: MODEL_EVIDENCE.MODEL_DERIVED, limitations: ['No dynamic balance or walking controller is active.', 'No physical contact plant is active.', 'No hardware-validation claim is made.'] }),
  microduck: capabilityRecord({ backend: EXECUTION_BACKENDS.LEGACY, capability: 'legacy policy demonstrator', evidence: MODEL_EVIDENCE.MODEL_DERIVED, limitations: ['Approximate browser dynamics remain active until the matched model/controller migration is complete.', 'No RL-environment, locomotion, contact, or hardware-parity claim is made.'] }),
});

// LeKiwi is the one profile that still exposes a legacy source-plant workspace beside its physical
// one, so its capability follows the selected workspace. A legacy run must never carry the
// browser-mujoco/numerically-verified claim that belongs to the physical workspace.
export const LEKIWI_LEGACY_CAPABILITY = capabilityRecord({
  backend: EXECUTION_BACKENDS.LEGACY,
  capability: 'legacy mobile-manipulation preview',
  evidence: MODEL_EVIDENCE.MODEL_DERIVED,
  limitations: LEGACY_LIMITS,
});

export function physicsCapabilityFor(profileId, { physical = true } = {}) {
  if (profileId === 'lekiwi' && !physical) return LEKIWI_LEGACY_CAPABILITY;
  return PHYSICS_PREVIEW_CAPABILITIES[profileId] || capabilityRecord({ backend: EXECUTION_BACKENDS.LEGACY, capability: 'unsupported physical workspace', evidence: MODEL_EVIDENCE.MODEL_DERIVED, limitations: LEGACY_LIMITS });
}
export function capabilityLabel(record) { return `${record.backend} · ${record.evidence}`; }
