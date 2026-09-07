import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord } from './model-registry.js';

const LEGACY_LIMITS = Object.freeze([
  'Physical MuJoCo backend not yet active for this workspace.',
  'Task outcome may still depend on legacy modeled behavior.',
  'No hardware-validation claim is made.',
]);

export const PHYSICS_PREVIEW_CAPABILITIES = Object.freeze({
  so101: capabilityRecord({
    backend: EXECUTION_BACKENDS.LEGACY,
    capability: 'legacy manipulation preview; SO-101 is the first physical migration target',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: LEGACY_LIMITS,
  }),
  openarm: capabilityRecord({
    backend: EXECUTION_BACKENDS.LEGACY,
    capability: 'legacy bimanual manipulation preview',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: LEGACY_LIMITS,
  }),
  lekiwi: capabilityRecord({
    backend: EXECUTION_BACKENDS.LEGACY,
    capability: 'legacy mobile-manipulation preview',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: LEGACY_LIMITS,
  }),
  unitree: capabilityRecord({
    backend: EXECUTION_BACKENDS.LEGACY,
    capability: 'kinematic joint-pose inspection only',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: [
      'No dynamic balance or walking controller is active.',
      'No physical contact plant is active.',
      'No hardware-validation claim is made.',
    ],
  }),
  microduck: capabilityRecord({
    backend: EXECUTION_BACKENDS.LEGACY,
    capability: 'legacy policy demonstrator',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: [
      'Approximate browser dynamics remain active until the matched model/controller migration is complete.',
      'No RL-environment, locomotion, contact, or hardware-parity claim is made.',
    ],
  }),
});

export function physicsCapabilityFor(profileId) {
  return PHYSICS_PREVIEW_CAPABILITIES[profileId] || capabilityRecord({
    backend: EXECUTION_BACKENDS.LEGACY,
    capability: 'unsupported physical workspace',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: LEGACY_LIMITS,
  });
}

export function capabilityLabel(record) {
  return `${record.backend} · ${record.evidence}`;
}
