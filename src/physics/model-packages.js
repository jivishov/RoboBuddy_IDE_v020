import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';

export const PHASE1_MODEL_PACKAGE = registerModelPackage({
  id: 'phase1-vertical-slice',
  robotId: 'phase1_articulated_joint',
  modelId: 'phase1-vertical-slice-v1',
  source: {
    url: 'https://github.com/jivishov/RoboBuddy_IDE_v020/blob/0fce3ee17fa605e4de63d53be255656984abccac/models/vertical-slice/model.xml',
    revision: '0fce3ee17fa605e4de63d53be255656984abccac',
    variant: 'Phase 1 conformance fixture',
  },
  license: 'MIT (repository-authored fixture)',
  asset: 'models/vertical-slice/model.xml',
  sha256: 'c8bfd81bb212afb88fd3ca173fc1235f1c1b45a79c3db0563ac22330ae7dab26',
  physics: { timestepSeconds: 0.002, integrator: 'RK4' },
  controllers: ['hinge_position'],
  joints: [{ id: 'hinge', evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }],
  actuators: [{ id: 'hinge_position', jointId: 'hinge', controllerId: 'hinge_position', command: 'position-rad' }],
  bodies: [{ id: 'free_box' }],
  sceneConstraints: { fixtures: ['floor', 'table'], objects: ['free_box'] },
  evidence: {
    geometry: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    inertia: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    actuators: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  },
  limitations: ['Minimal conformance fixture; not a production robot or hardware validation.'],
});
