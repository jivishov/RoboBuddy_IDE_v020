import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import { OPENARM_PHASE5A_MODEL_PACKAGE, OPENARM_PHASE5A_MODEL_SHA256, OPENARM_V2_UPSTREAM_REVISION } from '../../src/physics/openarm-model-package.js';
import { OPENARM_PHASE5A_CONTROLLER, OPENARM_PHASE5A_SCENE } from '../../src/physics/openarm-scene.js';

assert.equal(OPENARM_V2_UPSTREAM_REVISION, 'a8c979629f2591ad035d99d338ce114969e6cddc');
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.robotId, 'openarm_v2_bimanual');
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.physics.timestepSeconds, 0.001);
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.physics.integrator, 'Euler');
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.joints.length, 18);
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.actuators.length, 16);
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.source.revision, OPENARM_V2_UPSTREAM_REVISION);
assert.equal(OPENARM_PHASE5A_SCENE.modelPackage, OPENARM_PHASE5A_MODEL_PACKAGE.id);
assert.equal(OPENARM_PHASE5A_SCENE.legacyTaskId, 'openarm-04-filtration-workcell');
assert.equal(OPENARM_PHASE5A_CONTROLLER.stages.length, 18);
assert.deepEqual(OPENARM_PHASE5A_MODEL_PACKAGE.joints.find((j) => j.id === 'openarm_left_joint1').axis, [0,1,0]);
assert.deepEqual(OPENARM_PHASE5A_MODEL_PACKAGE.joints.find((j) => j.id === 'openarm_right_joint1').axis, [0,-1,0]);
assert.deepEqual(OPENARM_PHASE5A_MODEL_PACKAGE.joints.find((j) => j.id === 'openarm_left_joint2').rangeRad, [-3.3161,0.17453]);
assert.deepEqual(OPENARM_PHASE5A_MODEL_PACKAGE.joints.find((j) => j.id === 'openarm_right_joint2').rangeRad, [-0.17453,3.3161]);
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.actuators.some((a) => a.jointId === 'openarm_left_finger_joint2'), false, 'passive left finger must not become a second command channel');
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.actuators.some((a) => a.jointId === 'openarm_right_finger_joint2'), false, 'passive right finger must not become a second command channel');
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.initialJointPositionsRad.openarm_left_finger_joint1, OPENARM_PHASE5A_MODEL_PACKAGE.initialJointPositionsRad.openarm_left_finger_joint2);
assert.equal(OPENARM_PHASE5A_MODEL_PACKAGE.initialJointPositionsRad.openarm_right_finger_joint1, OPENARM_PHASE5A_MODEL_PACKAGE.initialJointPositionsRad.openarm_right_finger_joint2);
assert.ok(OPENARM_PHASE5A_MODEL_PACKAGE.limitations.some((line) => line.includes('lifter is fixed')));
assert.ok(OPENARM_PHASE5A_MODEL_PACKAGE.limitations.some((line) => line.includes('primitive collision surrogates')));

const xmlPath = new URL('../../models/openarm_v2/phase5a.xml', import.meta.url);
const xml = fs.readFileSync(xmlPath, 'utf8');
const digest = crypto.createHash('sha256').update(xml).digest('hex');
assert.equal(digest, OPENARM_PHASE5A_MODEL_SHA256);
assert.equal(digest, OPENARM_PHASE5A_MODEL_PACKAGE.sha256);
assert.equal((xml.match(/<freejoint /g) || []).length, 2, 'flask and beaker must remain true free bodies');
assert.equal((xml.match(/<weld\b/g) || []).length, 0, 'task grasp welds are forbidden');
assert.equal((xml.match(/<joint name="openarm_.*_finger_joint_mimic"/g) || []).length, 2, 'only source finger mechanical couplings are expected');
for (const token of ['phase5a_flask_geom','phase5a_beaker_geom','left_inner_finger_pad','left_outer_finger_pad','right_inner_finger_pad','right_outer_finger_pad','hotplate_top','wire_gauze_support']) assert.ok(xml.includes(token), `missing ${token}`);
for (const token of ['frictionloss="0.2" damping="1.0" armature="0.0081"','frictionloss="0.1" damping="0.9" armature="0.1600"','frictionloss="0.04" damping="0.9" armature="0.0100"','frictionloss="0.01" damping="0.01" armature="0.0049"','cone="elliptic" impratio="10"']) assert.ok(xml.includes(token), `source V2 dynamics token missing: ${token}`);

const stageNames = OPENARM_PHASE5A_CONTROLLER.stages.map((stage) => stage.name);
assert.deepEqual(stageNames, ['settle_initial','left_approach','left_close','left_lift','left_transfer','left_lower','left_release','left_settle','left_retreat','right_approach','right_close','right_lift','right_transfer','right_lower','right_release','right_settle','right_retreat','final_settle']);
assert.equal(OPENARM_PHASE5A_CONTROLLER.stages.find((s) => s.name === 'left_close').targetsRad.openarm_left_finger_joint1, 0);
assert.equal(OPENARM_PHASE5A_CONTROLLER.stages.find((s) => s.name === 'right_close').targetsRad.openarm_right_finger_joint1, 0);
assert.ok(OPENARM_PHASE5A_CONTROLLER.stages.every((stage) => Number.isFinite(stage.durationSeconds) && stage.durationSeconds > 0));

const simulator = fs.readFileSync(new URL('../../src/physics/openarm-physical-simulator.js', import.meta.url), 'utf8');
for (const forbidden of ['data.qpos', 'data.qvel', 'attachedTo', '.attach(', '.teleport(', '.move_to(']) assert.equal(simulator.includes(forbidden), false, `OpenArm physical simulator contains forbidden authority path ${forbidden}`);
assert.ok(simulator.includes("this.rig.root.position.set(185, 790, 0)"), 'presentation rig must align to source cell home mount');
assert.ok(simulator.includes('renderFrame()'));
assert.ok(!simulator.includes('mj_step'), 'renderer must never step MuJoCo');

const evaluator = fs.readFileSync(new URL('../../src/physics/openarm-task-evaluator.js', import.meta.url), 'utf8');
for (const token of ['linearVelocityMPerS','angularVelocityRadPerS','orderViolation','releaseSeen','retreatSeen','currentSupportContact']) assert.ok(evaluator.includes(token), `evaluator missing ${token}`);
for (const forbidden of ['success = true', 'attachedTo', 'teleport']) assert.equal(evaluator.includes(forbidden), false, `evaluator contains synthetic success path ${forbidden}`);

console.log('OpenArm Phase 5A model/controller/evaluator contract: OK');
