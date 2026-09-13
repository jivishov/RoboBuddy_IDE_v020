import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE, OPENARM_V2_SOURCE } from '../../src/physics/openarm-model-package.js';
import { OPENARM_V2_PHASE5A_SCENE, OPENARM_V2_BIMANUAL_CONTROLLER } from '../../src/physics/openarm-scene.js';
import { normalizeOpenArmLabEquipment, OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION } from '../../src/physics/openarm-lab-equipment.js';
import { OpenArmBimanualStackEvaluator } from '../../src/physics/openarm-task-evaluator.js';
import { createOpenArmPhysicalControlSchema, WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION } from '../../src/webmcp/openarm-physical-control.js';
import { createOpenArmPhysicalProgramSchema, WEBMCP_OPENARM_PROGRAM_SCHEMA_VERSION } from '../../src/webmcp/openarm-physical-program.js';
import { createOpenArmLabEquipmentSchema } from '../../src/webmcp/openarm-lab-equipment.js';

assert.equal(OPENARM_V2_SOURCE.revision, 'a8c979629f2591ad035d99d338ce114969e6cddc');
assert.equal(OPENARM_V2_SOURCE.sourcePath, 'v2/openarm_bimanual.xml');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.id, 'openarm-v2-phase5a-a8c9796-v2');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.modelId, 'robobuddy-openarm-v2-phase5a-v2');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.robotId, 'openarm_v2_bimanual');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.timestepSeconds, 0.001);
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.integrator, 'Euler');
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.joints.length, 16);
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.actuators.length, 16);
assert.equal(OPENARM_V2_PHASE5A_MODEL_PACKAGE.evidence.hardwareAlignment, 'calibration-required');
assert.equal(OPENARM_V2_PHASE5A_SCENE.modelPackage, OPENARM_V2_PHASE5A_MODEL_PACKAGE.id);
assert.equal(OPENARM_V2_PHASE5A_SCENE.revision, 'phase5a-openarm-v2-bimanual-stack-v3');
assert.equal(OPENARM_V2_PHASE5A_SCENE.taskGoal.maxGripPenetrationM, 0.004);
assert.equal(OPENARM_V2_BIMANUAL_CONTROLLER.id, 'openarm-v2-bimanual-stack-v2');
assert.equal(OPENARM_V2_BIMANUAL_CONTROLLER.stages.length, 15);
assert.equal(OPENARM_V2_BIMANUAL_CONTROLLER.stages[0].name, 'settle_initial');
assert.equal(OPENARM_V2_BIMANUAL_CONTROLLER.stages.at(-1).name, 'right_retreat');

const xml = fs.readFileSync(new URL('../../models/openarm_v2/manipulation.xml', import.meta.url), 'utf8');
assert.equal(createHash('sha256').update(xml).digest('hex'), OPENARM_V2_PHASE5A_MODEL_PACKAGE.sha256, 'OpenArm model package SHA must pin the reviewed MJCF exactly');
assert.ok(xml.includes('openarm_left_ee_finger_joint_mimic'));
assert.ok(xml.includes('openarm_right_ee_finger_joint_mimic'));
assert.equal((xml.match(/<weld\b/g) || []).length, 0, 'task model must contain no weld constraints');
assert.equal((xml.match(/<freejoint\b/g) || []).length, 2, 'flask and beaker must remain true free bodies');
assert.equal((xml.match(/size="0.008" class="left_fingertip"/g) || []).length, 2, 'left fingertip surrogate must remain the validated 8 mm approximation');
assert.equal((xml.match(/size="0.008" class="right_fingertip"/g) || []).length, 2, 'right fingertip surrogate must remain the validated 8 mm approximation');
assert.equal((xml.match(/fromto="0 0 -0.005 0 0 -0.075"/g) || []).length, 4, 'validated fingertip capsule centerlines must remain at the known-good baseline until exact source collision meshes replace them');
assert.ok(xml.includes('flask_shoulder_geom'), 'source-informed Erlenmeyer shoulder collider must be present');
assert.ok(xml.includes('size="0.025 0.030" class="vessel"'), '50 mL beaker must use the pinned 50 x 60 mm envelope');
assert.deepEqual(OPENARM_V2_PHASE5A_SCENE.taskGoal.flask.targetHalfExtentsXYM, [0.017, 0.013]);
assert.deepEqual(OPENARM_V2_PHASE5A_SCENE.taskGoal.beaker.targetHalfExtentsXYM, [0.021, 0.021]);
assert.deepEqual(OPENARM_V2_PHASE5A_SCENE.taskGoal.flask.forbiddenRobotGeoms, ['left_ee_proxy']);
assert.deepEqual(OPENARM_V2_PHASE5A_SCENE.taskGoal.beaker.forbiddenRobotGeoms, ['right_ee_proxy']);
for (const forbidden of ['attachedTo', 'teleport', 'move_to', 'grasp_right', 'grasp_left']) assert.equal(xml.includes(forbidden), false, `forbidden physical shortcut leaked into model: ${forbidden}`);

const schema = createOpenArmPhysicalControlSchema();
assert.equal(schema.oneOf.length, 2);
assert.equal(schema.oneOf[0].properties.schema_version.const, WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION);
assert.equal(schema.oneOf[0].additionalProperties, false);
assert.equal(schema.oneOf[1].additionalProperties, false);
assert.ok(schema.oneOf[0].properties.targets_rad.properties.openarm_left_joint1);
assert.ok(schema.oneOf[0].properties.targets_rad.properties.openarm_right_finger_joint1);

const programSchema = createOpenArmPhysicalProgramSchema();
assert.equal(programSchema.properties.schema_version.const, WEBMCP_OPENARM_PROGRAM_SCHEMA_VERSION);
assert.equal(programSchema.properties.segments.maxItems, 24);
assert.equal(programSchema.additionalProperties, false);
assert.ok(programSchema.properties.segments.items.properties.targets_rad.properties.openarm_left_joint1);

const equipmentSchema = createOpenArmLabEquipmentSchema();
assert.equal(equipmentSchema.oneOf.length, 4);
assert.equal(equipmentSchema.oneOf[0].properties.schema_version.const, OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION);
const normalizedEquipment = normalizeOpenArmLabEquipment([
  { id: 'fixture', shape: 'box', mobility: 'fixed', positionM: [0.62, 0.30, 1.025], sizeM: [0.12, 0.10, 0.04] },
  { id: 'vial', shape: 'cylinder', mobility: 'free', positionM: [0.50, 0.30, 1.055], sizeM: [0.012, 0.08], massKg: 0.025 },
]);
assert.equal(normalizedEquipment.length, 2);
assert.equal(normalizedEquipment[1].massKg, 0.025);
assert.throws(() => normalizeOpenArmLabEquipment([{ id: 'buried', shape: 'box', mobility: 'fixed', positionM: [0.50, 0.30, 1.01], sizeM: [0.10, 0.10, 0.04] }]), /intersects the physical tabletop/);
assert.throws(() => normalizeOpenArmLabEquipment([{ id: 'edge', shape: 'box', mobility: 'fixed', positionM: [0.78, 0.30, 1.03], sizeM: [0.20, 0.10, 0.04] }]), /beyond the physical tabletop in X/);

const obs = ({
  time,
  flask = [0.509, 0.1535, 1.092],
  beaker = [0.509, -0.1535, 1.105],
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
const flaskInner = { geom1Name: 'flask_grip_geom', geom2Name: 'left_inner_fingertip', distanceM: -0.001 };
const flaskOuter = { geom1Name: 'flask_grip_geom', geom2Name: 'left_outer_fingertip', distanceM: -0.001 };
const flaskSupport = { geom1Name: 'flask_body_geom', geom2Name: 'left_hotplate', distanceM: -0.0001 };
const beakerSupport = { geom1Name: 'beaker_grip_geom', geom2Name: 'right_ring_gauze', distanceM: -0.0001 };

const evaluator = new OpenArmBimanualStackEvaluator();
evaluator.observe(obs({
  time: 0,
  flask: [0.608, 0.1535, 1.092],
  beaker: [0.608, -0.1535, 1.105],
  contacts: [flaskSupport, beakerSupport],
}));
assert.equal(evaluator.snapshot().success, false);
assert.equal(evaluator.snapshot().flask.releaseSeen, false);
assert.equal(evaluator.snapshot().beaker.releaseSeen, false);

const sequentialFingerContact = new OpenArmBimanualStackEvaluator();
sequentialFingerContact.observe(obs({ time: 0, contacts: [flaskInner] }));
sequentialFingerContact.observe(obs({ time: 0.01, contacts: [flaskOuter] }));
assert.equal(sequentialFingerContact.snapshot().flask.innerContactSeen, true);
assert.equal(sequentialFingerContact.snapshot().flask.outerContactSeen, true);
assert.equal(sequentialFingerContact.snapshot().flask.graspSeen, false);
assert.equal(sequentialFingerContact.snapshot().flask.bilateralContactObservationCount, 0);

const palmEmbedded = new OpenArmBimanualStackEvaluator();
palmEmbedded.observe(obs({ time: 0, contacts: [flaskInner, flaskOuter, { geom1Name: 'flask_grip_geom', geom2Name: 'left_ee_proxy', distanceM: -0.001 }] }));
assert.equal(palmEmbedded.snapshot().flask.forbiddenRobotContactSeen, true);
assert.equal(palmEmbedded.snapshot().flask.graspClearanceValid, false);
assert.equal(palmEmbedded.snapshot().flask.graspSeen, false);

const deeplyEmbedded = new OpenArmBimanualStackEvaluator();
deeplyEmbedded.observe(obs({ time: 0, contacts: [
  { ...flaskInner, distanceM: -0.005 },
  { ...flaskOuter, distanceM: -0.005 },
] }));
assert.equal(deeplyEmbedded.snapshot().flask.maxGripPenetrationM, 0.005);
assert.equal(deeplyEmbedded.snapshot().flask.graspClearanceValid, false);
assert.equal(deeplyEmbedded.snapshot().flask.graspSeen, false);

const droppedIntoTarget = new OpenArmBimanualStackEvaluator();
droppedIntoTarget.observe(obs({ time: 0, contacts: [flaskInner, flaskOuter] }));
droppedIntoTarget.observe(obs({ time: 0.10, flask: [0.509, 0.1535, 1.120], contacts: [flaskInner, flaskOuter] }));
droppedIntoTarget.observe(obs({ time: 0.20, flask: [0.579, 0.1535, 1.120], contacts: [flaskInner, flaskOuter] }));
assert.equal(droppedIntoTarget.snapshot().flask.carrySeen, true);
droppedIntoTarget.observe(obs({ time: 0.30, flask: [0.608, 0.1535, 1.092], contacts: [] }));
droppedIntoTarget.observe(obs({ time: 0.40, flask: [0.608, 0.1535, 1.092], contacts: [flaskSupport] }));
assert.equal(droppedIntoTarget.snapshot().flask.supportWhileHeldSeen, false);
assert.equal(droppedIntoTarget.snapshot().flask.releaseSeen, false);

const noFakeRetreat = new OpenArmBimanualStackEvaluator();
noFakeRetreat.observe(obs({ time: 0, contacts: [flaskInner, flaskOuter] }));
noFakeRetreat.observe(obs({ time: 0.10, flask: [0.509, 0.1535, 1.120], contacts: [flaskInner, flaskOuter] }));
noFakeRetreat.observe(obs({ time: 0.20, flask: [0.579, 0.1535, 1.120], contacts: [flaskInner, flaskOuter] }));
noFakeRetreat.observe(obs({ time: 0.30, flask: [0.608, 0.1535, 1.092], contacts: [flaskInner, flaskOuter, flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.supportWhileHeldSeen, true);
noFakeRetreat.observe(obs({ time: 0.31, flask: [0.608, 0.1535, 1.092], contacts: [flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.releaseSeen, true);
noFakeRetreat.observe(obs({ time: 0.51, flask: [0.608, 0.1535, 1.092], contacts: [flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.settled, true);
noFakeRetreat.observe(obs({ time: 0.52, flask: [0.608, 0.1535, 1.092], contacts: [flaskSupport], leftEe: [0.608, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.retreated, false);
noFakeRetreat.observe(obs({ time: 0.60, flask: [0.608, 0.1535, 1.092], contacts: [flaskSupport], leftEe: [0.668, 0.1535, 1.155] }));
assert.equal(noFakeRetreat.snapshot().flask.retreated, true);

const OPENARM_MEASURED_MIN_CAUSAL_WINDOW_STEPS = 3;
const simulatorSource = fs.readFileSync(new URL('../../src/physics/openarm-physical-simulator.js', import.meta.url), 'utf8');
const declaredCadence = Number(simulatorSource.match(/const OPENARM_OBSERVATION_BATCH_STEPS = (\d+);/)?.[1]);
assert.ok(Number.isInteger(declaredCadence) && declaredCadence >= 1, 'the OpenArm workspace must declare an integer observation cadence in physics steps');
assert.ok(
  declaredCadence < OPENARM_MEASURED_MIN_CAUSAL_WINDOW_STEPS,
  `observation cadence ${declaredCadence} steps cannot guarantee observing the measured ${OPENARM_MEASURED_MIN_CAUSAL_WINDOW_STEPS}-step support-while-held overlap`,
);
assert.ok(simulatorSource.includes('observationBatchSteps: OPENARM_OBSERVATION_BATCH_STEPS'), 'the declared cadence must be the one the physical session actually uses');
assert.ok(simulatorSource.includes('observationPeriodSeconds:'), 'the presentation audit must expose the observation period actually in force');
assert.ok(simulatorSource.includes("plane = 'physical-tabletop'"), 'the presentation grid must describe the physical tabletop rather than a detached world floor');
assert.ok(simulatorSource.includes('this.presentationDirty = true;'), 'observations must mark presentation stale rather than drive the scene graph per sample');
assert.ok(/renderFrame\(\) \{[\s\S]*?this\.#applyObservation\(this\.lastObservation\);/.test(simulatorSource), 'the render loop must pull the latest observed state instead of physics pushing it');
assert.ok(!/#consumeObservation\(observation\) \{[\s\S]*?this\.#applyObservation\(observation\);/.test(simulatorSource), 'per-sample scene-graph updates must not be reintroduced');

console.log('OpenArm V2 Phase 5A package/evaluator/WebMCP core checks: OK');