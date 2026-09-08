import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { OPENARM_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';

export const OPENARM_PHASE5A_SCENE = Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id: 'phase5a-openarm-bimanual-stack',
  revision: 'phase5a-openarm-bimanual-stack-v1',
  robotId: OPENARM_PHASE5A_MODEL_PACKAGE.robotId,
  modelPackage: OPENARM_PHASE5A_MODEL_PACKAGE.id,
  legacyTaskId: 'openarm-04-filtration-workcell',
  physics: Object.freeze({
    timestepSeconds: OPENARM_PHASE5A_MODEL_PACKAGE.physics.timestepSeconds,
    integrator: OPENARM_PHASE5A_MODEL_PACKAGE.physics.integrator,
  }),
  fixtures: Object.freeze([
    Object.freeze({ id: 'cell_table_col' }),
    Object.freeze({ id: 'hotplate_top' }),
    Object.freeze({ id: 'wire_gauze_support' }),
  ]),
  objects: Object.freeze([
    Object.freeze({ id: 'phase5a_flask' }),
    Object.freeze({ id: 'phase5a_beaker' }),
  ]),
  controllers: Object.freeze([...OPENARM_PHASE5A_MODEL_PACKAGE.controllers]),
  taskGoal: Object.freeze({
    type: 'openarm-sequential-bimanual-supported-stack',
    order: Object.freeze(['phase5a_flask', 'phase5a_beaker']),
    requireRealGripperContact: true,
    requireLift: true,
    requireHeldTransport: true,
    requireRelease: true,
    requireSupportContact: true,
    requireStableRest: true,
    requireRetreat: true,
  }),
});

const stage = (name, durationSeconds, targetsRad = {}) => Object.freeze({ name, durationSeconds, targetsRad: Object.freeze(targetsRad) });

export const OPENARM_PHASE5A_CONTROLLER = Object.freeze({
  id: OPENARM_PHASE5A_MODEL_PACKAGE.benchmark.controllerVersion,
  controllerPeriodSeconds: 0.02,
  stages: Object.freeze([
    stage('settle_initial', 0.20),
    stage('left_approach', 0.60, { openarm_left_joint1: 0.30, openarm_left_joint2: 0.0 }),
    stage('left_close', 0.40, { openarm_left_finger_joint1: 0.0 }),
    stage('left_lift', 0.70, { openarm_left_joint1: 0.15, openarm_left_joint2: -0.20 }),
    stage('left_transfer', 0.80, { openarm_left_joint1: 0.0, openarm_left_joint2: -0.40 }),
    stage('left_lower', 0.60, { openarm_left_joint1: 0.275, openarm_left_joint2: -0.40 }),
    stage('left_release', 0.40, { openarm_left_finger_joint1: 0.45 }),
    stage('left_settle', 0.40),
    stage('left_retreat', 0.40, { openarm_left_joint1: 0.0, openarm_left_joint2: -0.40 }),
    stage('right_approach', 0.60, { openarm_right_joint1: -0.30, openarm_right_joint2: 0.0 }),
    stage('right_close', 0.40, { openarm_right_finger_joint1: 0.0 }),
    stage('right_lift', 0.70, { openarm_right_joint1: -0.15, openarm_right_joint2: 0.20 }),
    stage('right_transfer', 0.80, { openarm_right_joint1: 0.0, openarm_right_joint2: 0.40 }),
    stage('right_lower', 0.60, { openarm_right_joint1: -0.10, openarm_right_joint2: 0.40 }),
    stage('right_release', 0.40, { openarm_right_finger_joint1: -0.45 }),
    stage('right_settle', 0.40),
    stage('right_retreat', 0.40, { openarm_right_joint1: 0.0, openarm_right_joint2: 0.40 }),
    stage('final_settle', 0.40),
  ]),
});
