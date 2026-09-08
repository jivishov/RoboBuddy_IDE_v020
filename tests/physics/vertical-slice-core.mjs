import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BrowserMuJoCoBackend } from '../../src/physics/browser-mujoco-backend.js';
import { assertPhysicalScene, makeCommandEnvelope, PHYSICS_BACKEND_API_VERSION } from '../../src/physics/backend-contract.js';
import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord, requireModelPackage } from '../../src/physics/model-registry.js';
import '../../src/physics/model-packages.js';
import { PHASE1_SCENE } from '../../src/physics/phase1-scene.js';
import { PhysicsSession } from '../../src/physics/session.js';

const requiredMethods = ['loadScene', 'reset', 'acceptCommand', 'advanceSteps', 'getObservation', 'getDiagnostics', 'pause', 'resume', 'cancelRun', 'exportTrace', 'dispose'];
for (const method of requiredMethods) assert.equal(typeof BrowserMuJoCoBackend.prototype[method], 'function', `${method} must exist on BrowserMuJoCoBackend`);
const capability = capabilityRecord({ backend: EXECUTION_BACKENDS.BROWSER_MUJOCO, capability: 'phase1-vertical-slice', evidence: MODEL_EVIDENCE.MODEL_DERIVED, limitations: ['Not hardware validation', 'Not a migrated production robot'] });
assert.equal(capability.backend, 'browser-mujoco'); assert.equal(capability.evidence, 'model-derived');
const phase1Package = requireModelPackage(PHASE1_SCENE.modelPackage);
assert.equal(phase1Package.robotId, PHASE1_SCENE.robotId); assert.equal(phase1Package.physics.timestepSeconds, PHASE1_SCENE.physics.timestepSeconds); assert.equal(phase1Package.physics.integrator, PHASE1_SCENE.physics.integrator);
assert.deepEqual(phase1Package.sceneConstraints.fixtures, PHASE1_SCENE.fixtures.map(({ id }) => id)); assert.deepEqual(phase1Package.sceneConstraints.objects, PHASE1_SCENE.objects.map(({ id }) => id));
assert.doesNotThrow(() => assertPhysicalScene(structuredClone(PHASE1_SCENE)));
assert.throws(() => assertPhysicalScene({ revision: 'partial', robotId: 'robot' }), /schemaVersion/);
assert.throws(() => assertPhysicalScene({ ...structuredClone(PHASE1_SCENE), unexpected: true }), /Unknown physical scene field/);
assert.throws(() => assertPhysicalScene({ ...structuredClone(PHASE1_SCENE), physics: { ...PHASE1_SCENE.physics, timestepSeconds: 0 } }), /timestepSeconds/);
assert.throws(() => makeCommandEnvelope({ sessionId: 's', epoch: 0, commandId: 'c', sceneRevision: 'r', robotId: 'robot', command: { type: 'x' } }), /positive integer/);
assert.throws(() => makeCommandEnvelope({ sessionId: 's', epoch: 1, commandId: 'c', sceneRevision: 'r', robotId: 'robot', command: { type: 'x' }, maxSteps: 0 }), /maxSteps/);

class FakeBackend {
  constructor() { this.calls = []; this.loaded = false; }
  async loadScene(scene, context) { this.calls.push(['loadScene', scene, context]); this.loaded = true; return { sceneRevision: scene.revision, robotId: scene.robotId, observation: { simulationTimeSeconds: 0 } }; }
  async reset(context) { this.calls.push(['reset', context]); return { simulationTimeSeconds: 0 }; }
  async acceptCommand(envelope) { this.calls.push(['acceptCommand', envelope]); return { status: 'accepted', commandId: envelope.commandId }; }
  async advanceSteps(steps, context) { this.calls.push(['advanceSteps', steps, context]); return { simulationTimeSeconds: steps * 0.002 }; }
  async getObservation(context) { this.calls.push(['getObservation', context]); return { simulationTimeSeconds: 0 }; }
  async getDiagnostics(context) { this.calls.push(['getDiagnostics', context]); return { backend: 'fake' }; }
  async pause(context) { this.calls.push(['pause', context]); return true; }
  async resume(context) { this.calls.push(['resume', context]); return true; }
  async cancelRun(context) { this.calls.push(['cancelRun', context]); this.loaded = false; return { cancelled: true, reloadRequired: true }; }
  async exportTrace(context) { this.calls.push(['exportTrace', context]); return { events: [] }; }
  dispose() { this.calls.push(['dispose']); }
}

assert.throws(() => new PhysicsSession(new FakeBackend(), { observationBatchSteps: 0 }), /positive integer/);

const fake = new FakeBackend(); const session = new PhysicsSession(fake, { sessionId: 'phase1-test-session' });
await assert.rejects(() => session.loadScene({ revision: 'partial', robotId: 'phase1_articulated_joint' }), /schemaVersion/);
const loaded = await session.loadScene(structuredClone(PHASE1_SCENE)); assert.equal(loaded.apiVersion, PHYSICS_BACKEND_API_VERSION); assert.equal(loaded.epoch, 1); assert.equal(fake.calls.at(-1)[2].epoch, 1);
assert.equal(fake.calls.at(-1)[1].modelPackage, PHASE1_SCENE.modelPackage); assert.equal(fake.calls.at(-1)[1].physics.timestepSeconds, 0.002); assert.equal(fake.calls.at(-1)[1].physics.integrator, 'RK4');
await session.sendCommand({ type: 'set_joint_target', targetRad: 0.4 }, { commandId: 'cmd-1', maxSteps: 50 });
const envelope = fake.calls.at(-1)[1]; assert.equal(envelope.sessionId, 'phase1-test-session'); assert.equal(envelope.epoch, 1); assert.equal(envelope.sceneRevision, PHASE1_SCENE.revision); assert.equal(envelope.robotId, PHASE1_SCENE.robotId); assert.equal(envelope.maxSteps, 50);
await session.reset(); assert.equal(session.epoch, 2); assert.equal(fake.calls.at(-1)[1].epoch, 2); await session.advanceSteps(10); assert.equal(fake.calls.at(-1)[2].epoch, 2);
const cancelled = await session.cancelRun('test-cancel'); assert.deepEqual(cancelled, { cancelled: true, reloadRequired: true }); assert.equal(session.robotId, null); assert.equal(session.sceneRevision, null);
await assert.rejects(() => session.sendCommand({ type: 'set_joint_target', targetRad: 0 }), /No physical scene loaded/);

const sampledFake = new FakeBackend();
const sampledSession = new PhysicsSession(sampledFake, { sessionId: 'sampled-advance-session', observationBatchSteps: 3 });
await sampledSession.loadScene(structuredClone(PHASE1_SCENE));
const sampledEvents = [];
const unsubscribeSampled = sampledSession.subscribe((event) => sampledEvents.push(event));
const sampledFinal = await sampledSession.advanceSteps(8);
const sampledCalls = sampledFake.calls.filter(([type]) => type === 'advanceSteps');
assert.deepEqual(sampledCalls.map(([, steps]) => steps), [3, 3, 2], 'long advances must be executed as deterministic physics-step observation batches');
assert.equal(sampledEvents.length, 3, 'each physics observation batch must publish actual backend state');
assert.equal(sampledEvents.every((event) => event.source === 'advanceSteps'), true);
assert.equal(sampledEvents.every((event) => event.sessionId === 'sampled-advance-session' && event.epoch === 1), true);
assert.deepEqual(sampledFinal, sampledEvents.at(-1).observation, 'advanceSteps must return the final authoritative observation');
unsubscribeSampled();
sampledSession.dispose();

class LostAuthorityBackend extends FakeBackend {
  async advanceSteps(steps, context) {
    this.calls.push(['advanceSteps', steps, context]);
    this.loaded = false;
    throw new Error('runtime plant failure');
  }
  async getDiagnostics(context) {
    this.calls.push(['getDiagnostics', context]);
    return { backend: 'fake', loaded: this.loaded };
  }
}

const lostBackend = new LostAuthorityBackend();
const lostSession = new PhysicsSession(lostBackend, { sessionId: 'lost-authority-session' });
await lostSession.loadScene(structuredClone(PHASE1_SCENE));
await lostSession.sendCommand({ type: 'set_joint_target', targetRad: 0.3 }, { commandId: 'before-runtime-failure', maxSteps: 20 });
assert.equal(lostSession.activeCommandId, 'before-runtime-failure');
await assert.rejects(() => lostSession.advanceSteps(1), /runtime plant failure/);
assert.equal(lostSession.robotId, null, 'fatal backend authority loss must clear the session robot');
assert.equal(lostSession.sceneRevision, null, 'fatal backend authority loss must clear the session scene revision');
assert.equal(lostSession.activeCommandId, null, 'fatal backend authority loss must clear the accepted command identity');
await assert.rejects(() => lostSession.advanceSteps(1), /No physical scene loaded/);
lostSession.dispose();

class RejectedCommandBackend extends FakeBackend {
  async acceptCommand(envelope) {
    this.calls.push(['acceptCommand', envelope]);
    throw new RangeError('target outside declared range');
  }
  async getDiagnostics(context) {
    this.calls.push(['getDiagnostics', context]);
    return { backend: 'fake', loaded: this.loaded };
  }
}

const rejectedBackend = new RejectedCommandBackend();
const rejectedSession = new PhysicsSession(rejectedBackend, { sessionId: 'rejected-command-session' });
await rejectedSession.loadScene(structuredClone(PHASE1_SCENE));
await assert.rejects(() => rejectedSession.sendCommand({ type: 'set_joint_target', targetRad: 99 }, { commandId: 'rejected-command', maxSteps: 20 }), /outside declared range/);
assert.equal(rejectedSession.robotId, PHASE1_SCENE.robotId, 'ordinary command rejection must not discard a valid physical plant');
assert.equal(rejectedSession.sceneRevision, PHASE1_SCENE.revision);
assert.equal(rejectedSession.activeCommandId, null, 'a rejected command must never become the active accepted command');
rejectedSession.dispose();

const workerSource = readFileSync(new URL('../../src/physics/mujoco-worker.js', import.meta.url), 'utf8');
for (const token of ['mj_name2id', 'jnt_qposadr', 'jnt_dofadr', 'actuator_trnid', 'data.xpos', "crypto.subtle.digest('SHA-256'", 'descriptor.joints', 'descriptor.actuators']) assert.ok(workerSource.includes(token), `worker must use named/address/model-descriptor access: ${token}`);
for (const forbidden of ['data.qpos?.[0]', 'data.qvel?.[0]', 'data.ctrl[0]', "'hinge_position'", "'free_box'", "'models/vertical-slice/model.xml'"]) assert.ok(!workerSource.includes(forbidden), `generic worker must not rely on Phase 1-specific model semantics: ${forbidden}`);
assert.ok(workerSource.includes('collection.delete?.()'), 'worker must release the copied MuJoCo contact vector'); assert.ok(workerSource.includes('contact.delete?.()'), 'worker must release copied MuJoCo contact handles');
assert.ok(workerSource.includes('is outside the actuator control range'), 'worker must reject out-of-range targets instead of silently clamping'); assert.ok(workerSource.includes('Worker rejected non-registry model asset path'), 'worker must reject arbitrary model URLs/paths');
const backendSource = readFileSync(new URL('../../src/physics/browser-mujoco-backend.js', import.meta.url), 'utf8');
assert.ok(backendSource.includes('executedSteps'), 'backend must account actual executed physics steps');
assert.ok(backendSource.includes('remainingSteps -= executedSteps'), 'paused/no-op requests must not consume unexecuted command steps');
assert.ok(backendSource.includes('requireModelPackage(scene.modelPackage)'), 'backend must resolve scenes through the controlled model registry');
assert.ok(!backendSource.includes('PHASE1_SCENE'), 'generic backend must not be intrinsically tied to the Phase 1 scene');
assert.ok(backendSource.includes('#failLoadedScene(reason)'), 'backend must centralize fatal runtime invalidation');
assert.ok(backendSource.includes("this.#failLoadedScene('runtime step failed')"), 'fatal stepping faults must invalidate physical authority');
assert.ok(backendSource.includes('else this.#failLoadedScene(reason)'), 'worker crashes must invalidate physical authority');
session.dispose(); console.log('Phase 1 vertical-slice contract checks: OK');
