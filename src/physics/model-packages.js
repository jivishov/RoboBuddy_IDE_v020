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

const SO101_JOINTS = [
  ['shoulder_pan', [-1.91986, 1.91986]],
  ['shoulder_lift', [-1.7453293, 1.7453293]],
  ['elbow_flex', [-1.69, 1.69]],
  ['wrist_flex', [-1.658063, 1.658063]],
  ['wrist_roll', [-2.7438473, 2.7438473]],
  ['gripper', [-0.174533, 1.7453292]],
];

export const SO101_PHASE2A_MODEL_PACKAGE = registerModelPackage({
  id: 'so101-phase2a-menagerie-8161bba',
  robotId: 'so101_follower',
  modelId: 'robobuddy-so101-phase2a-v1',
  source: {
    url: 'https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/so101.xml',
    revision: '8161bba264d7fa7c99ca301e91e7fb44737676ad',
    upstreamRevision: 'aec17bbc256d1a7342d53aaa4950595d4c30b40d',
    variant: 'The Robot Studio SO-101 follower arm',
  },
  license: 'Apache-2.0',
  asset: 'models/so101/model.xml',
  sha256: 'ebac47fac296cddff086c513bc1c7e50ca2ddc3dce0c774c388a13f24e3178b9',
  physics: { timestepSeconds: 0.005, integrator: 'implicitfast', iterations: 10, lsIterations: 20 },
  controllers: ['so101_position'],
  joints: SO101_JOINTS.map(([id, rangeRad]) => ({ id, rangeRad, axis: [0, 0, 1], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED })),
  actuators: SO101_JOINTS.map(([id]) => ({ id, jointId: id, controllerId: 'so101_position', command: 'position-rad', evidence: PARAMETER_EVIDENCE.ESTIMATED })),
  bodies: [{ id: 'base' }, { id: 'gripper' }, { id: 'moving_jaw_so101_v1' }],
  evidence: {
    kinematics: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    inertia: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    collisionPrimitives: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    servoControllerParameters: PARAMETER_EVIDENCE.ESTIMATED,
    hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  },
  limitations: [
    'Phase 2A articulated-plant validation package only; normal IDE capability is not promoted yet.',
    'Upstream visual meshes and mesh-only gripper collisions are omitted from this self-contained browser validation MJCF.',
    'Servo gains/force settings are upstream simulation estimates, not hardware calibration.',
    'No grasp/block-transfer task or hardware-fidelity claim in Phase 2A.',
  ],
});
