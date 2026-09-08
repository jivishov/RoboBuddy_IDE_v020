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
  joints: [{ id: 'hinge', rangeRad: [-1.57, 1.57], axis: [0, 1, 0], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }],
  actuators: [{ id: 'hinge_position', jointId: 'hinge', controllerId: 'hinge_position', command: 'position-rad', controlRangeRad: [-1.2, 1.2], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED }],
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

const SO101_CONTROL_RANGES = Object.freeze({
  shoulder_pan: [-1.91986, 1.91986],
  shoulder_lift: [-1.74533, 1.74533],
  elbow_flex: [-1.69, 1.69],
  wrist_flex: [-1.65806, 1.65806],
  wrist_roll: [-2.74385, 2.84121],
  gripper: [-0.17453, 1.74533],
});

export const SO101_PHASE2A_MODEL_PACKAGE = registerModelPackage({
  id: 'so101-phase2a-menagerie-8161bba',
  robotId: 'so101_follower',
  modelId: 'robobuddy-so101-phase2a-v1',
  source: {
    url: 'https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/so101.xml',
    revision: '8161bba264d7fa7c99ca301e91e7fb44737676ad',
    upstreamRevision: 'aec17bbc256d1a7342d53aaa4950595d4c30b40d',
    variant: 'Pinned MuJoCo Menagerie SO-101 simulation adaptation of The Robot Studio follower arm; Phase 2A self-contained adaptation omits the source camera-mount child',
  },
  license: 'Apache-2.0',
  asset: 'models/so101/model.xml',
  sha256: 'e23653717a5cc44f0c706cf73098398f3ea1fac167e0560eef5d2f579834bdd0',
  physics: { timestepSeconds: 0.005, integrator: 'implicitfast', iterations: 10, lsIterations: 20 },
  controllers: ['so101_position'],
  joints: SO101_JOINTS.map(([id, rangeRad]) => ({ id, rangeRad, axis: [0, 0, 1], evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED })),
  actuators: SO101_JOINTS.map(([id]) => ({ id, jointId: id, controllerId: 'so101_position', command: 'position-rad', controlRangeRad: SO101_CONTROL_RANGES[id], evidence: PARAMETER_EVIDENCE.ESTIMATED })),
  bodies: [{ id: 'base' }, { id: 'gripper' }, { id: 'moving_jaw_so101_v1' }],
  evidence: {
    kinematics: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    actuatedLinkInertias: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    retainedCollisionPrimitives: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    servoControllerParameters: PARAMETER_EVIDENCE.ESTIMATED,
    cameraMountDynamics: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
    hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  },
  limitations: [
    'Phase 2A articulated-plant validation package only; normal IDE capability is not promoted yet.',
    'The pinned Menagerie wrist_roll upper joint limit is 2.7438473 rad, while The Robot Studio source revision recorded by Menagerie uses about 2.8412063 rad; RoboBuddy preserves the pinned Menagerie simulation constraint and does not present it as a measured hardware limit.',
    'Upstream visual meshes, mesh gripper collisions, the camera-mount child and some nonessential source collision geometry are omitted from this self-contained browser validation MJCF.',
    'The omitted upstream camera-mount child carries a source mesh mass of 0.012 kg, so this adapted plant must not be described as dynamically identical to the full pinned Menagerie model.',
    'Servo gains/force settings are upstream simulation estimates, not hardware calibration.',
    'No grasp/block-transfer task or hardware-fidelity claim in Phase 2A.',
  ],
});
