import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';

export const OPENARM_V2_SOURCE = Object.freeze({
  repository: 'enactic/openarm_mujoco',
  revision: 'a8c979629f2591ad035d99d338ce114969e6cddc',
  sourcePath: 'v2/openarm_bimanual.xml',
  sourceBlob: '0bd77d3bf7e0a5f3d2361fdf5e8328d00d0b2cc9',
  cellPath: 'v2/cell/cell.xml',
  variant: 'OpenArm V2 bimanual plant mounted with the V2 cell origin relationship',
});

const joint = (id, rangeRad, axis) => ({ id, rangeRad, axis, evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED });
const actuator = (id, jointId, rangeRad) => ({
  id,
  jointId,
  controllerId: 'openarm_v2_position',
  command: 'position-rad',
  controlRangeRad: rangeRad,
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
});

const joints = Object.freeze([
  joint('openarm_left_joint1', [-3.4907, 1.3963], [0, 1, 0]),
  joint('openarm_left_joint2', [-3.3161, 0.17453], [-1, 0, 0]),
  joint('openarm_left_joint3', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_left_joint4', [0, 2.4435], [0, -1, 0]),
  joint('openarm_left_joint5', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_left_joint6', [-0.7854, 0.7854], [0, -1, 0]),
  joint('openarm_left_joint7', [-1.5708, 1.5708], [1, 0, 0]),
  joint('openarm_left_finger_joint1', [0, 0.7854], [-1, 0, 0]),
  joint('openarm_right_joint1', [-1.3963, 3.4907], [0, -1, 0]),
  joint('openarm_right_joint2', [-0.17453, 3.3161], [-1, 0, 0]),
  joint('openarm_right_joint3', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_right_joint4', [0, 2.4435], [0, -1, 0]),
  joint('openarm_right_joint5', [-1.5708, 1.5708], [0, 0, -1]),
  joint('openarm_right_joint6', [-0.7854, 0.7854], [0, 1, 0]),
  joint('openarm_right_joint7', [-1.5708, 1.5708], [1, 0, 0]),
  joint('openarm_right_finger_joint1', [-0.7854, 0], [-1, 0, 0]),
]);

const actuators = Object.freeze([
  actuator('left_joint1_ctrl', 'openarm_left_joint1', [-3.49066, 1.39626]),
  actuator('left_joint2_ctrl', 'openarm_left_joint2', [-3.31613, 0.174533]),
  actuator('left_joint3_ctrl', 'openarm_left_joint3', [-1.5708, 1.5708]),
  actuator('left_joint4_ctrl', 'openarm_left_joint4', [0, 2.44346]),
  actuator('left_joint5_ctrl', 'openarm_left_joint5', [-1.5708, 1.5708]),
  actuator('left_joint6_ctrl', 'openarm_left_joint6', [-0.785398, 0.785398]),
  actuator('left_joint7_ctrl', 'openarm_left_joint7', [-1.5708, 1.5708]),
  actuator('left_finger1_ctrl', 'openarm_left_finger_joint1', [0, 0.7854]),
  actuator('right_joint1_ctrl', 'openarm_right_joint1', [-1.39626, 3.49066]),
  actuator('right_joint2_ctrl', 'openarm_right_joint2', [-0.174533, 3.31613]),
  actuator('right_joint3_ctrl', 'openarm_right_joint3', [-1.5708, 1.5708]),
  actuator('right_joint4_ctrl', 'openarm_right_joint4', [0, 2.44346]),
  actuator('right_joint5_ctrl', 'openarm_right_joint5', [-1.5708, 1.5708]),
  actuator('right_joint6_ctrl', 'openarm_right_joint6', [-0.785398, 0.785398]),
  actuator('right_joint7_ctrl', 'openarm_right_joint7', [-1.5708, 1.5708]),
  actuator('right_finger1_ctrl', 'openarm_right_finger_joint1', [-0.7854, 0]),
]);

const mechanicalCouplings = Object.freeze([
  Object.freeze({
    id: 'openarm_left_ee_finger_joint_mimic',
    driverJointId: 'openarm_left_finger_joint1',
    followerJointId: 'openarm_left_finger_joint2',
    multiplier: 1,
    offsetRad: 0,
    evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  }),
  Object.freeze({
    id: 'openarm_right_ee_finger_joint_mimic',
    driverJointId: 'openarm_right_finger_joint1',
    followerJointId: 'openarm_right_finger_joint2',
    multiplier: 1,
    offsetRad: 0,
    evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  }),
]);

const armBodies = [];
for (const side of ['left', 'right']) {
  armBodies.push({ id: `openarm_${side}_base_link` });
  for (let index = 1; index <= 6; index += 1) armBodies.push({ id: `openarm_${side}_link${index}` });
  armBodies.push({ id: `openarm_${side}_ee_base_link` });
  armBodies.push({ id: `openarm_${side}_ee_inner_finger` });
  armBodies.push({ id: `openarm_${side}_ee_outer_finger` });
}

export const OPENARM_V2_PHASE5A_MODEL_PACKAGE = registerModelPackage({
  id: 'openarm-v2-phase5a-a8c9796-v1',
  robotId: 'openarm_v2_bimanual',
  modelId: 'robobuddy-openarm-v2-phase5a-v1',
  source: {
    url: `https://github.com/${OPENARM_V2_SOURCE.repository}/blob/${OPENARM_V2_SOURCE.revision}/${OPENARM_V2_SOURCE.sourcePath}`,
    revision: OPENARM_V2_SOURCE.revision,
    variant: `${OPENARM_V2_SOURCE.variant}; repository-local primitive collision adaptation for browser execution`,
  },
  license: 'Apache-2.0 for OpenArm-derived plant; repository-authored dry benchmark fixtures are MIT',
  asset: 'models/openarm_v2/manipulation.xml',
  sha256: '323a345733dc45a4b3d7616e83ea5aef48cb099eb29855691530252b167f64b5',
  physics: { timestepSeconds: 0.001, integrator: 'Euler' },
  controllers: ['openarm_v2_position'],
  joints,
  actuators,
  mechanicalCouplings,
  bodies: [
    ...armBodies,
    { id: 'flask', freeJointId: 'flask_free' },
    { id: 'beaker', freeJointId: 'beaker_free' },
  ],
  initialJointPositionsRad: {
    openarm_left_joint1: 0,
    openarm_left_joint2: 0,
    openarm_left_joint3: 0,
    openarm_left_joint4: Math.PI / 2,
    openarm_left_joint5: 0,
    openarm_left_joint6: 0,
    openarm_left_joint7: 0,
    openarm_left_finger_joint1: 0.65,
    openarm_right_joint1: 0,
    openarm_right_joint2: 0,
    openarm_right_joint3: 0,
    openarm_right_joint4: Math.PI / 2,
    openarm_right_joint5: 0,
    openarm_right_joint6: 0,
    openarm_right_joint7: 0,
    openarm_right_finger_joint1: -0.65,
  },
  sceneConstraints: {
    fixtures: ['cell_table', 'left_source_support', 'left_hotplate', 'right_source_support', 'right_ring_post', 'right_ring_gauze'],
    objects: ['flask', 'beaker'],
  },
  benchmark: {
    mountOriginM: [0.185, 0, 1.34],
    sourceHomeEeM: {
      left: [0.401, 0.1535, 1.12],
      right: [0.401, -0.1535, 1.12],
    },
    sourceSupports: {
      flaskTopZM: 1.035,
      beakerTopZM: 1.075,
    },
    destinationSupports: {
      flask: { id: 'left_hotplate', centerXYM: [0.608, 0.1535], topZM: 1.035, halfExtentsXYM: [0.045, 0.045] },
      beaker: { id: 'right_ring_gauze', centerXYM: [0.608, -0.1535], topZM: 1.075, halfExtentsXYM: [0.040, 0.040] },
    },
    objects: {
      flask: { massKg: 0.060, initialPositionM: [0.509, 0.1535, 1.085], supportBottomZM: 1.035 },
      beaker: { massKg: 0.050, initialPositionM: [0.509, -0.1535, 1.120], supportBottomZM: 1.075 },
    },
    controllerVersion: 'openarm-v2-bimanual-stack-v1',
  },
  evidence: {
    kinematicsAndMirroredFrames: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    linkMassAndInertia: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    dampingArmatureFrictionLoss: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    actuatorGainsAndForceLimits: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    mechanicalFingerCoupling: PARAMETER_EVIDENCE.SOURCE_DERIVED,
    primitiveCollisionSurrogates: PARAMETER_EVIDENCE.ESTIMATED,
    dryTaskFixtureAndVesselGeometry: PARAMETER_EVIDENCE.ESTIMATED,
    hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  },
  limitations: [
    'The plant preserves the pinned OpenArm V2 kinematic tree, mirrored axes/ranges, inertials and source simulation actuator semantics, but replaces upstream mesh collision geometry with explicit primitive surrogates for a self-contained browser package.',
    'Primitive collision surrogates are estimated and intentionally do not support claims about exact self-collision margins, fingertip pressure distribution, glass compliance, or installed-hardware clearances.',
    'The hotplate, ring stand/gauze, staging supports, flask and beaker are controlled dry benchmark geometry. Their dimensions, masses and friction are not measured laboratory hardware.',
    'Source actuator gains and force limits are simulator parameters from the pinned V2 model, not calibration of a particular assembled OpenArm.',
    'The only equality constraints are the source-derived left/right finger mechanical couplings. Passive finger2 qpos is initialized consistently with the coupled actuated finger1 at setup/reset; it is never exposed as a separate command surface.',
    'There is no object grasp weld, parent attachment, snap, teleport, or task-success state overwrite.',
    'No thermal, electrical, liquid, force-sensor, tactile-sensor, CAN timing, backlash, compliance, or hardware-safety validation is claimed.',
  ],
});
