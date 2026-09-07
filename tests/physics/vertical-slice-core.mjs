import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BrowserMuJoCoBackend } from '../../src/physics/browser-mujoco-backend.js';
import { makeCommandEnvelope, PHYSICS_BACKEND_API_VERSION } from '../../src/physics/backend-contract.js';
import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord } from '../../src/physics/model-registry.js';
import { PhysicsSession } from '../../src/physics/session.js';

const requiredMethods = [
  'loadScene', 'reset', 'acceptCommand', 'advanceSteps', 'getObservation',
  'getDiagnostics', 'pause', 'resume', 'cancelRun', 'exportTrace', 'dispose',
];
for (const method of requiredMethods) {
  assert.equal(typeof BrowserMuJoCoBackend.prototype[method], 'function', `${method} must exist on BrowserMuJoCoBackend`);
}

const capability = capabilityRecord({
  backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
  capability: 'phase1-vertical-slice',
  evidence: MODEL_EVIDENCE.MODEL_DERIVED,
  limitations: ['Not hardware validation', 'Not a migrated production robot'],
});
assert.equal(capability.backend, 'browser-mujoco');
assert.equal(capability.evidence, 'model-derived');

assert.throws(() => makeCommandEnvelope({
  sessionId: 's', epoch: 0, commandId: 'c', sceneRevision: 'r', robotId: 'robot', command: { type: 'x' },
}), /positive integer/);
assert.throws(() => makeCommandEnvelope({
  sessionId: 's', epoch: 1, commandId: 'c', sceneRevision: 'r', robotId: 'robot', command: { type: 'x' }, maxSteps: 0,
}), /maxSteps/);

class FakeBackend {
  constructor() {
    this.calls = [];
    this.loaded = false;
  }
  async loadScene(scene, context) {
    this.calls.push(['loadScene', scene, context]);
    this.loaded = true;
    return { sceneRevision: scene.revision, robotId: scene.robotId, observation: { simulationTime: 0 } };
  }
  async reset(context) { this.calls.push(['reset', context]); return { simulationTime: 0 }; }
  async acceptCommand(envelope) { this.calls.push(['acceptCommand', envelope]); return { status: 'accepted', commandId: envelope.commandId }; }
  async advanceSteps(steps, context) { this.calls.push(['advanceSteps', steps, context]); return { simulationTime: steps * 0.002 }; }
  async getObservation(context) { this.calls.push(['getObservation', context]); return { simulationTime: 0 }; }
  async getDiagnostics(context) { this.calls.push(['getDiagnostics', context]); return { backend: 'fake' }; }
  async pause(context) { this.calls.push(['pause', context]); return true; }
  async resume(context) { this.calls.push(['resume', context]); return true; }
  async cancelRun(context) { this.calls.push(['cancelRun', context]); this.loaded = false; return { cancelled: true, reloadRequired: true }; }
  async exportTrace(context) { this.calls.push(['exportTrace', context]); return { events: [] }; }
  dispose() { this.calls.push(['dispose']); }
}

const fake = new FakeBackend();
const session = new PhysicsSession(fake, { sessionId: 'phase1-test-session' });
const loaded = await session.loadScene({ revision: 'phase1-v1', robotId: 'phase1_articulated_joint' });
assert.equal(loaded.apiVersion, PHYSICS_BACKEND_API_VERSION);
assert.equal(loaded.epoch, 1);
assert.equal(fake.calls.at(-1)[2].epoch, 1);

await session.sendCommand({ type: 'set_joint_target', targetRad: 0.4 }, { commandId: 'cmd-1', maxSteps: 50 });
const envelope = fake.calls.at(-1)[1];
assert.equal(envelope.sessionId, 'phase1-test-session');
assert.equal(envelope.epoch, 1);
assert.equal(envelope.sceneRevision, 'phase1-v1');
assert.equal(envelope.robotId, 'phase1_articulated_joint');
assert.equal(envelope.maxSteps, 50);

await session.reset();
assert.equal(session.epoch, 2);
assert.equal(fake.calls.at(-1)[1].epoch, 2);
await session.advanceSteps(10);
assert.equal(fake.calls.at(-1)[2].epoch, 2);

const cancelled = await session.cancelRun('test-cancel');
assert.deepEqual(cancelled, { cancelled: true, reloadRequired: true });
assert.equal(session.robotId, null);
assert.equal(session.sceneRevision, null);
await assert.rejects(() => session.sendCommand({ type: 'set_joint_target', targetRad: 0 }), /No physical scene loaded/);

const workerSource = readFileSync(new URL('../../src/physics/mujoco-worker.js', import.meta.url), 'utf8');
for (const token of ['mj_name2id', 'jnt_qposadr', 'jnt_dofadr', 'actuator_trnid', 'data.xpos']) {
  assert.ok(workerSource.includes(token), `worker must use named/address-table access: ${token}`);
}
for (const forbidden of ['data.qpos?.[0]', 'data.qvel?.[0]', 'data.ctrl[0]']) {
  assert.ok(!workerSource.includes(forbidden), `worker must not rely on hard-coded model offsets: ${forbidden}`);
}
assert.ok(workerSource.includes('targetRad ${target} is outside the actuator control range'), 'worker must reject out-of-range targets instead of silently clamping');

session.dispose();
console.log('Phase 1 vertical-slice contract checks: OK');
