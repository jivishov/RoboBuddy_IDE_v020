import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';

const d2r = (degrees) => degrees * Math.PI / 180;

const armTargets = (side, valuesDeg, fingerRad = null) => {
  const targets = {};
  valuesDeg.forEach((value, index) => { targets[`openarm_${side}_joint${index + 1}`] = d2r(value); });
  if (fingerRad != null) targets[`openarm_${side}_finger_joint1`] = fingerRad;
  return Object.freeze(targets);
};

const LEFT_HOME = [0, 0, 0, 90, 0, 0, 0];
const RIGHT_HOME = [0, 0, 0, 90, 0, 0, 0];
const LEFT_LIFT = [-0.545, 0, 0, 97.436, 0, -7.941, 0];
const RIGHT_LIFT = [0.545, 0, 0, 97.436, 0, 7.941, 0];
const LEFT_TRANSFER = [-26.7699, 0, 0, 64.934, 0, -1.6954, 0];
const RIGHT_TRANSFER = [26.7699, 0, 0, 64.934, 0, 1.6954, 0];
const LEFT_PLACE = [-27.1394, 0, 0, 56.4231, 0, 6.4055, 0];
const RIGHT_PLACE = [27.1394, 0, 0, 56.4231, 0, -6.4055, 0];

export const OPENARM_V2_PHASE5A_SCENE = Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id: 'phase5a-openarm-v2-bimanual-stack',
  revision: 'phase5a-openarm-v2-bimanual-stack-v2',
  robotId: OPENARM_V2_PHASE5A_MODEL_PACKAGE.robotId,
  modelPackage: OPENARM_V2_PHASE5A_MODEL_PACKAGE.id,
  legacyTaskId: 'openarm-04-filtration-workcell',
  physics: Object.freeze({
    timestepSeconds: OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.timestepSeconds,
    integrator: OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.integrator,
  }),
  fixtures: Object.freeze(OPENARM_V2_PHASE5A_MODEL_PACKAGE.sceneConstraints.fixtures.map((id) => Object.freeze({ id }))),
  objects: Object.freeze(OPENARM_V2_PHASE5A_MODEL_PACKAGE.sceneConstraints.objects.map((id) => Object.freeze({ id }))),
  controllers: Object.freeze([...OPENARM_V2_PHASE5A_MODEL_PACKAGE.controllers]),
  taskGoal: Object.freeze({
    type: 'openarm-v2-bimanual-dry-stack',
    order: Object.freeze(['flask', 'beaker']),
    flask: Object.freeze({
      objectId: 'flask',
      side: 'left',
      gripperGeoms: Object.freeze(['left_inner_fingertip', 'left_outer_fingertip']),
      objectGeoms: Object.freeze(['flask_body_geom', 'flask_shoulder_geom', 'flask_grip_geom']),
      supportGeom: 'left_hotplate',
      targetCenterXYM: Object.freeze([0.608, 0.1535]),
      targetHalfExtentsXYM: Object.freeze([0.017, 0.013]),
      initialBodyZM: 1.092,
    }),
    beaker: Object.freeze({
      objectId: 'beaker',
      side: 'right',
      gripperGeoms: Object.freeze(['right_inner_fingertip', 'right_outer_fingertip']),
      objectGeoms: Object.freeze(['beaker_grip_geom']),
      supportGeom: 'right_ring_gauze',
      targetCenterXYM: Object.freeze([0.608, -0.1535]),
      targetHalfExtentsXYM: Object.freeze([0.021, 0.021]),
      initialBodyZM: 1.105,
    }),
    liftClearanceM: 0.020,
    carryHorizontalM: 0.060,
    settleSeconds: 0.20,
    maxSettleDriftM: 0.001,
    maxSettleLinearSpeedMS: 0.035,
    maxSettleAngularSpeedRadS: 0.8,
    retreatDistanceM: 0.050,
  }),
});

export const OPENARM_V2_BIMANUAL_CONTROLLER = Object.freeze({
  id: 'openarm-v2-bimanual-stack-v2',
  controllerPeriodSeconds: 0.02,
  stages: Object.freeze([
    Object.freeze({ name: 'settle_initial', label: 'Settle both free vessels on their source supports', durationSeconds: 0.40, targetsRad: Object.freeze({}) }),
    Object.freeze({ name: 'left_close', label: 'Left gripper establishes a maintained bilateral flask pinch', durationSeconds: 0.50, targetsRad: Object.freeze({ openarm_left_finger_joint1: 0.05 }) }),
    Object.freeze({ name: 'left_lift', label: 'Left arm lifts the contacted flask', durationSeconds: 1.00, targetsRad: armTargets('left', LEFT_LIFT, 0.05) }),
    Object.freeze({ name: 'left_transfer', label: 'Left arm carries the flask toward the unpowered hotplate', durationSeconds: 1.20, targetsRad: armTargets('left', LEFT_TRANSFER, 0.05) }),
    Object.freeze({ name: 'left_lower', label: 'Left arm lowers the held flask onto the hotplate', durationSeconds: 0.90, targetsRad: armTargets('left', LEFT_PLACE, 0.05) }),
    Object.freeze({ name: 'left_release', label: 'Left gripper opens after hotplate support contact', durationSeconds: 0.45, targetsRad: Object.freeze({ openarm_left_finger_joint1: 0.65 }) }),
    Object.freeze({ name: 'left_settle', label: 'Allow the released flask to settle under gravity/contact', durationSeconds: 0.45, targetsRad: Object.freeze({}) }),
    Object.freeze({ name: 'left_retreat', label: 'Left arm retreats clear of the settled flask', durationSeconds: 0.85, targetsRad: armTargets('left', LEFT_LIFT, 0.65) }),
    Object.freeze({ name: 'right_close', label: 'Right gripper establishes a maintained bilateral beaker pinch', durationSeconds: 0.50, targetsRad: Object.freeze({ openarm_right_finger_joint1: -0.33 }) }),
    Object.freeze({ name: 'right_lift', label: 'Right arm lifts the contacted beaker', durationSeconds: 1.00, targetsRad: armTargets('right', RIGHT_LIFT, -0.33) }),
    Object.freeze({ name: 'right_transfer', label: 'Right arm carries the beaker toward the ring stand', durationSeconds: 1.20, targetsRad: armTargets('right', RIGHT_TRANSFER, -0.33) }),
    Object.freeze({ name: 'right_lower', label: 'Right arm lowers the held beaker onto the wire gauze', durationSeconds: 0.90, targetsRad: armTargets('right', RIGHT_PLACE, -0.33) }),
    Object.freeze({ name: 'right_release', label: 'Right gripper opens after gauze support contact', durationSeconds: 0.45, targetsRad: Object.freeze({ openarm_right_finger_joint1: -0.65 }) }),
    Object.freeze({ name: 'right_settle', label: 'Allow the released beaker to settle under gravity/contact', durationSeconds: 0.45, targetsRad: Object.freeze({}) }),
    Object.freeze({ name: 'right_retreat', label: 'Right arm retreats clear of the settled beaker', durationSeconds: 0.85, targetsRad: armTargets('right', RIGHT_LIFT, -0.65) }),
  ]),
  referencePoses: Object.freeze({
    sourceHomeEeM: OPENARM_V2_PHASE5A_MODEL_PACKAGE.benchmark.sourceHomeEeM,
    leftLiftEeM: Object.freeze([0.401, 0.1535, 1.150]),
    leftTransferEeM: Object.freeze([0.500, 0.1535, 1.150]),
    leftPlaceEeM: Object.freeze([0.500, 0.1535, 1.120]),
    rightLiftEeM: Object.freeze([0.401, -0.1535, 1.150]),
    rightTransferEeM: Object.freeze([0.500, -0.1535, 1.150]),
    rightPlaceEeM: Object.freeze([0.500, -0.1535, 1.120]),
  }),
});
