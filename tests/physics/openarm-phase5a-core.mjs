import assert from 'node:assert/strict';
import fs from 'node:fs';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE, OPENARM_V2_SOURCE } from '../../src/physics/openarm-model-package.js';
import { OPENARM_V2_PHASE5A_SCENE, OPENARM_V2_BIMANUAL_CONTROLLER } from '../../src/physics/openarm-scene.js';
import { OpenArmBimanualStackEvaluator } from '../../src/physics/openarm-task-evaluator.js';
import { createOpenArmPhysicalControlSchema, WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION } from '../../src/webmcp/openarm-physical-control.js';

assert.equal(OPENARM_V2_SOURCE.revision, 'a8c979629f2591ad035d99d338ce114969e6cddc');
assert.equal(OPENARM_V2_SOURCE.sourcePath, 'v2/openarm_bimanual.xml');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.robotId, 'openarm_v2_bimanual');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.timestepSeconds, 0.001);
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.integrator, 'Euler');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.joints.length, 16);
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.actuators.length, 16);
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.evidence.hardwareAlignment, 'calibration-required');
assert.equal(OPENARM_V2_PHASE5A_SCENE.modelPackage, OPENARM_V2_PHASE5A_MODEL_PACKAGE.id);
assert.equal(OPENARM_V2_BIMANUAL_CONTROLLER.stages.length, 15);
assert.equal(OPENARM_V2_BIMANUAL_CONTROLLER.stages[0].name, 'settle_initial');
assert.equal(OPENARM_V2_BIMANUAL_CONTROLLER.stages.at(-1).name, 'right_retreat');

const xml = fs.readFileSync(new URL('../../models/openarm_v2/manipulation.xml', import.meta.url), 'utf8');
assert.ok(xml.includes('openarm_left_ee_finger_joint_mimic'));
assert.ok(xml.includes('openarm_right_ee_finger_joint_mimic'));
assert.equal((xml.match(/<weld\b/g) || []).length, 0, 'task model must contain no weld constraints');
assert.equal((xml.match(/<freejoint\b/g) || []).length, 2, 'flask and beaker must remain true free bodies');
for (const forbidden of ['attachedTo', 'teleport', 'move_to', 'grasp_right', 'grasp_left']) assert.equal(xml.includes(forbidden), false, `forbidden physical shortcut leaked into model: ${forbidden}`);

const schema = createOpenArmPhysicalControlSchema();
assert.equal(schema.oneOf.length, 2);
assert.equal(schema.oneOf[0].properties.schema_version.const, WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION);
assert.equal(schema.oneOf[0].additionalProperties, false);
assert.equal(schema.oneOf[1].additionalProperties, false);
assert.ok(schema.oneOf[0].properties.targets_rad.properties.openarm_left_joint1);
assert.ok(schema.oneOf[0].properties.targets_rad.properties.openarm_right_finger_joint1);

const obs = ({
  time,
  flask = [0.509, 0.1535, 1.085],
  beaker = [0.509, -0.1535, 1.120],
  contacts = [],
  leftEe = [0.40, 0.1535, 1.12],
  rightEe = [0.40, -0.1535, 1.12],
  flaskLinear = [0, 0, 0],
  flaskAngular = [0, 0, 0],
  beakerLinear = [0, 0, 0],
  beakerAngular = [0, 0, 0],
}) => ({
  simulationTimeSeconds: time,
  bodies: {
    flask: { positionM: flask, linearVelocityMS: flaskLinear, angularVelocityRadS: flaskAngular },
    beaker: { positionM: beaker, linearVelocityMS: beakerLinear, angularVelocityRadS: beakerAngular },
    openarm_left_ee_base_link: { positionM: leftEe },
    openarm_right_ee_base_link: { positionM: rightEe },
  },
  contacts,
});
const flaskInner = { geom1Name: 'flask_grip_geom', geom2Name: 'left_inner_fingertip' };
const flaskOuter = { geom1Name: 'flask_grip_geom', geom2Name: 'left_outer_fingertip' };
const flaskSupport = { geom1Name: 'flask_body_geom', geom2Name: 'left_hotplate' };
const beakerSupport = { geom1Name: 'beaker_grip_geom', geom2Name: 'right_ring_gauze' };

// Target inclusion/support contact without a physical grasp/lift/carry cannot pass.
const evaluator = new OpenArmBimanualStackEvaluator();
evaluator.observe(obs({
  time: 0,
  flask: [0.608, 0.1535, 1.085],
  beaker: [0.608, -0.1535, 1.120],
  contacts: [flaskSupport, beakerSupport],
}));
assert.equal(evaluator.snapshot().success, false);
assert.equal(evaluator.snapshot().flask.releaseSeen, false);
assert.equal(evaluator.snapshot().beaker.releaseSeen, false);

// Contacts that occur on opposite fingers at different times are not a bilateral grasp.
const sequentialFingerContact = new OpenArmBimanualStackEvaluator();
sequentialFingerContact.observe(obs({ time: 0, contacts: [flaskInner] }));
sequentialFingerContact.observe(obs({ time: 0.01, contacts: [flaskOuter] }));
assert.equal(sequentialFingerContact.snapshot().flask.innerContactSeen, true);
assert.equal(sequentialFingerContact.snapshot().flask.outerContactSeen, true);
assert.equal(sequentialFingerContact.snapshot().flask.graspSeen, false);
assert.equal(sequentialFingerContact.snapshot().flask.bilateralContactObservationCount, 0);

// A vessel dropped before reaching its support may later land in the target, but that is not a controlled placement/release.
const droppedIntoTarget = new OpenArmBimanualStackEvaluator();
droppedIntoTarget.observe(obs({ time: 0, contacts: [flaskInner, flaskOuter] }));
droppedIntoTarget.observe(obs({ time: 0.10, flask: [0.509, 0.1535, 1.110], contacts: [flaskInner, flaskOuter] }));
droppedIntoTarget.observe(obs({ time: 0.20, flask: [0.579, 0.1535, 1.110], contacts: [flaskInner, flaskOuter] }));
assert.equal(droppedIntoTarget.snapshot().flask.carrySeen, true);
droppedIntoTarget.observe(obs({ time: 0.30, flask: [0.608, 0.1535, 1.085], contacts: [] }));
droppedIntoTarget.observe(obs({ time: 0.40, flask: [0.608, 0.1535, 1.085], contacts: [flaskSupport] }));
assert.equal(droppedIntoTarget.snapshot().flask.supportWhileHeldSeen, false);
assert.equal(droppedIntoTarget.snapshot().flask.releaseSeen, false);

// Retreat evidence is actual post-settle EE displacement; normal EE/object geometry separation cannot satisfy it by itself.
const noFakeRetreat = new OpenArmBimanualStackEvaluator();
noFakeRetreat.observe(obs({ time: 0, contacts: [flaskInner, flaskOuter] }));
noFakeRetreat.observe(obs({ time: 0.10, flask: [0.509, 0.1535, 1.110], contacts: [flaskInner, flaskOuter] }));
noFakeRetreat.observe(obs({ time: 0.20, flask: [0.579, 0.1535, 1.110], contacts: [flaskInner, flaskOuter] }));
noFakeRetreat.observe(obs({ time: 0.30, flask: [0.608, 0.1535, 1.085], contacts: [flaskInner, flaskOuter, flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.supportWhileHeldSeen, true);
noFakeRetreat.observe(obs({ time: 0.31, flask: [0.608, 0.1535, 1.085], contacts: [flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.releaseSeen, true);
noFakeRetreat.observe(obs({ time: 0.51, flask: [0.608, 0.1535, 1.085], contacts: [flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.settled, true);
noFakeRetreat.observe(obs({ time: 0.52, flask: [0.608, 0.1535, 1.085], contacts: [flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.retreated, false);
noFakeRetreat.observe(obs({ time: 0.60, flask: [0.608, 0.1535, 1.085], contacts: [flaskSupport], leftEe: [0.668, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.retreated, true);

console.log('OpenArm V2 Phase 5A package/evaluator/WebMCP core checks: OK');
