import { assertPhysicalScene, assertPhysicsBackend, makeCommandEnvelope, PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';

function uid(prefix) {
  return `${prefix}-${crypto.randomUUID?.() || Math.random().toString(36).slice(2)}`;
}

export class PhysicsSession {
  constructor(backend, { sessionId = uid('physics') } = {}) {
    this.backend = assertPhysicsBackend(backend);
    this.sessionId = String(sessionId);
    this.epoch = 0;
    this.sceneRevision = null;
    this.robotId = null;
    this.disposed = false;
    this.activeCommandId = null;
  }

  async loadScene(scene) {
    this.#assertLive();
    assertPhysicalScene(scene);
    this.epoch += 1;
    this.activeCommandId = null;
    try {
      const result = await this.backend.loadScene(structuredClone(scene), this.#context());
      this.sceneRevision = result?.sceneRevision ?? scene.revision;
      this.robotId = result?.robotId ?? scene.robotId;
      return { apiVersion: PHYSICS_BACKEND_API_VERSION, ...this.#context(), ...result };
    } catch (error) {
      this.#clearLoadedState();
      throw error;
    }
  }

  async reset(options = {}) {
    this.#assertLoaded();
    this.epoch += 1;
    this.activeCommandId = null;
    const result = await this.#runLoadedOperation(() => this.backend.reset({ ...structuredClone(options), ...this.#context() }));
    return { ...this.#context(), result };
  }

  async sendCommand(command, { commandId = uid('cmd'), maxSteps = null } = {}) {
    this.#assertLoaded();
    const envelope = makeCommandEnvelope({
      ...this.#context(),
      commandId,
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      command,
      maxSteps,
    });
    const result = await this.#runLoadedOperation(() => this.backend.acceptCommand(envelope));
    this.activeCommandId = envelope.commandId;
    return result;
  }

  async advanceSteps(steps) {
    this.#assertLoaded();
    if (!Number.isInteger(steps) || steps < 1) throw new RangeError('steps must be a positive integer');
    return this.#runLoadedOperation(() => this.backend.advanceSteps(steps, this.#context()));
  }

  async getObservation(options = {}) {
    this.#assertLoaded();
    return this.#runLoadedOperation(() => this.backend.getObservation({ ...options, ...this.#context() }));
  }

  async getDiagnostics() {
    this.#assertLive();
    const diagnostics = await this.backend.getDiagnostics(this.#context());
    if (diagnostics?.loaded === false) this.#clearLoadedState();
    return diagnostics;
  }

  async pause() {
    this.#assertLoaded();
    return this.#runLoadedOperation(() => this.backend.pause(this.#context()));
  }

  async resume() {
    this.#assertLoaded();
    return this.#runLoadedOperation(() => this.backend.resume(this.#context()));
  }

  async cancelRun(reason = 'cancelled') {
    this.#assertLoaded();
    this.epoch += 1;
    const result = await this.#runLoadedOperation(() => this.backend.cancelRun({ reason, ...this.#context() }));
    this.activeCommandId = null;
    if (result?.reloadRequired) this.#clearLoadedState();
    return result;
  }

  exportTrace() {
    this.#assertLive();
    return this.backend.exportTrace(this.#context());
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.epoch += 1;
    this.#clearLoadedState();
    this.backend.dispose();
  }

  #context() {
    return { sessionId: this.sessionId, epoch: this.epoch };
  }

  async #runLoadedOperation(operation) {
    try {
      return await operation();
    } catch (error) {
      try {
        const diagnostics = await this.backend.getDiagnostics(this.#context());
        if (diagnostics?.loaded === false) this.#clearLoadedState();
      } catch {
        // Preserve the original operation failure. A diagnostic failure is not authority evidence.
      }
      throw error;
    }
  }

  #clearLoadedState() {
    this.activeCommandId = null;
    this.sceneRevision = null;
    this.robotId = null;
  }

  #assertLoaded() {
    this.#assertLive();
    if (!this.robotId || !this.sceneRevision) throw new Error('No physical scene loaded');
  }

  #assertLive() {
    if (this.disposed) throw new Error('Physics session is disposed');
  }
}
