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

// Causal evaluator unit trace: target inclusion without bilateral grasp/lift/carry cannot pass.
const evaluator = new OpenArmBimanualStackEvaluator();
const obs = (time, flask, beaker, contacts = []) => ({
  simulationTimeSeconds: time,
  bodies: {
    flask: { positionM: flask, linearVelocityMS: [0,0,0], angularVelocityRadS: [0,0,0] },
    beaker: { positionM: beaker, linearVelocityMS: [0,0,0], angularVelocityRadS: [0,0,0] },
    openarm_left_ee_base_link: { positionM: [0.4,0.15,1.3] },
    openarm_right_ee_base_link: { positionM: [0.4,-0.15,1.3] },
  },
  contacts,
});
evaluator.observe(obs(0, [0.608,0.1535,1.085], [0.608,-0.1535,1.120], [
  { geom1Name: 'flask_body_geom', geom2Name: 'left_hotplate' },
  { geom1Name: 'beaker_grip_geom', geom2Name: 'right_ring_gauze' },
]));
assert.equal(evaluator.snapshot().success, false);
assert.equal(evaluator.snapshot().flask.releaseSeen, false);
assert.equal(evaluator.snapshot().beaker.releaseSeen, false);

console.log('OpenArm V2 Phase 5A package/evaluator/WebMCP core checks: OK');
