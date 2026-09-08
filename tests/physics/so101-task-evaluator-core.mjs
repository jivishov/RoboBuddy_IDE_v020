import assert from 'node:assert/strict';
import {
  So101BlockTransferEvaluator,
  hasSo101BlockGripperContact,
  hasSo101BlockTargetSupportContact,
} from '../../src/physics/so101-task-evaluator.js';

const SOURCE = [0.39416, -0.00169, 0.234];
const TARGET = [0.358, -0.156, 0.234];
const gripperContact = [{ geom1Name: 'benchmark_block_geom', geom2Name: 'fixed_jaw_box1' }];
const targetSupportContact = [{ geom1Name: 'benchmark_block_geom', geom2Name: 'benchmark_target_support' }];

function observation(time, position, contacts = []) {
  return {
    simulationTimeSeconds: time,
    bodies: { benchmark_block: { positionM: [...position] } },
    contacts: structuredClone(contacts),
  };
}

assert.equal(hasSo101BlockGripperContact(observation(0, SOURCE, gripperContact)), true);
assert.equal(hasSo101BlockTargetSupportContact(observation(0, TARGET, targetSupportContact)), true);

const evaluator = new So101BlockTransferEvaluator();
evaluator.observe(observation(0.0, SOURCE));
evaluator.observe(observation(1.3, SOURCE, gripperContact));
evaluator.observe(observation(2.1, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
evaluator.observe(observation(3.0, [TARGET[0], TARGET[1], 0.31], gripperContact));
evaluator.observe(observation(4.3, [TARGET[0], TARGET[1], 0.245]));
let state = evaluator.observe(observation(5.1, TARGET, targetSupportContact));
assert.equal(state.contactSeen, true);
assert.equal(state.liftSeen, true);
assert.equal(state.carrySeen, true);
assert.equal(state.releaseSeen, true);
assert.equal(state.settleSeen, true);
assert.equal(state.inTarget, true);
assert.equal(state.currentTargetSupportContact, true);
assert.equal(state.success, true, 'final success requires observed post-release support contact and simulation-time settling');

const outside = new So101BlockTransferEvaluator();
outside.observe(observation(0.0, SOURCE));
outside.observe(observation(1.0, SOURCE, gripperContact));
outside.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
outside.observe(observation(3.0, [0.42, -0.16, 0.31], gripperContact));
outside.observe(observation(4.0, [0.42, -0.16, 0.245]));
state = outside.observe(observation(5.0, [0.42, -0.16, 0.234], targetSupportContact));
assert.equal(state.inTarget, false);
assert.equal(state.success, false, 'support contact cannot substitute for final target inclusion');

const noCarry = new So101BlockTransferEvaluator();
noCarry.observe(observation(0.0, SOURCE));
noCarry.observe(observation(1.0, SOURCE, gripperContact));
noCarry.observe(observation(2.0, [SOURCE[0], SOURCE[1], 0.31], gripperContact));
noCarry.observe(observation(3.0, TARGET));
noCarry.observe(observation(4.0, TARGET, targetSupportContact));
assert.equal(noCarry.snapshot().carrySeen, false);
assert.equal(noCarry.snapshot().success, false, 'teleport-like target appearance without carried contact cannot succeed');

evaluator.reset();
state = evaluator.snapshot();
assert.equal(state.success, false);
assert.equal(state.contactSeen, false);
assert.equal(state.initialPositionM, null);

console.log('SO-101 observation-derived task evaluator: OK');
