import { assertPhysicsBackend, makeCommandEnvelope, PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';

function uid(prefix) {
  return `${prefix}-${crypto.randomUUID?.() || Math.random().toString(36).slice(2)}`;
}

export class PhysicsSession {
  constructor(backend, { sessionId = uid('physics') } = {}) {
    this.backend = assertPhysicsBackend(backend);
    this.sessionId = sessionId;
    this.epoch = 0;
    this.sceneRevision = null;
    this.robotId = null;
    this.disposed = false;
    this.activeCommandId = null;
  }

  async loadScene(scene) {
    this.#assertLive();
    this.epoch += 1;
    this.activeCommandId = null;
    const result = await this.backend.loadScene(structuredClone(scene), { sessionId: this.sessionId, epoch: this.epoch });
    this.sceneRevision = result?.sceneRevision ?? scene?.revision ?? null;
    this.robotId = result?.robotId ?? scene?.robotId ?? null;
    return { apiVersion: PHYSICS_BACKEND_API_VERSION, sessionId: this.sessionId, epoch: this.epoch, ...result };
  }

  async reset(options = {}) {
    this.#assertLive();
    this.epoch += 1;
    this.activeCommandId = null;
    return this.backend.reset({ ...structuredClone(options), sessionId: this.sessionId, epoch: this.epoch });
  }

  async sendCommand(command, { commandId = uid('cmd'), maxSteps = null } = {}) {
    this.#assertLive();
    if (!this.robotId) throw new Error('No physical scene loaded');
    const envelope = makeCommandEnvelope({
      sessionId: this.sessionId,
      epoch: this.epoch,
      commandId,
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      command,
      maxSteps,
    });
    this.activeCommandId = commandId;
    return this.backend.acceptCommand(envelope);
  }

  advanceSteps(steps) {
    this.#assertLive();
    if (!Number.isInteger(steps) || steps < 1) throw new RangeError('steps must be a positive integer');
    return this.backend.advanceSteps(steps, { sessionId: this.sessionId, epoch: this.epoch });
  }

  getObservation(options = {}) {
    this.#assertLive();
    return this.backend.getObservation({ ...options, sessionId: this.sessionId, epoch: this.epoch });
  }

  getDiagnostics() {
    this.#assertLive();
    return this.backend.getDiagnostics({ sessionId: this.sessionId, epoch: this.epoch });
  }

  pause() { this.#assertLive(); return this.backend.pause(); }
  resume() { this.#assertLive(); return this.backend.resume(); }

  async cancelRun(reason = 'cancelled') {
    this.#assertLive();
    this.epoch += 1;
    const cancelled = await this.backend.cancelRun({ reason, sessionId: this.sessionId, epoch: this.epoch });
    this.activeCommandId = null;
    return cancelled;
  }

  exportTrace() {
    this.#assertLive();
    return this.backend.exportTrace({ sessionId: this.sessionId, epoch: this.epoch });
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.epoch += 1;
    this.activeCommandId = null;
    this.backend.dispose();
  }

  #assertLive() {
    if (this.disposed) throw new Error('Physics session is disposed');
  }
}
