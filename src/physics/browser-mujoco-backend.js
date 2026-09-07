import { assertPhysicsBackend } from './backend-contract.js';

export class BrowserMuJoCoBackend {
  constructor({ workerUrl = new URL('./mujoco-worker.js', import.meta.url), modelUrl = new URL('../../models/vertical-slice/model.xml', import.meta.url) } = {}) {
    this.workerUrl = workerUrl;
    this.modelUrl = modelUrl;
    this.worker = null;
    this.sequence = 0;
    this.pending = new Map();
    this.loaded = false;
    this.disposed = false;
    this.sceneRevision = null;
    this.robotId = null;
    this.lastObservation = null;
  }

  async loadScene(scene = {}) {
    if (this.disposed) throw new Error('Backend is disposed');
    if (!this.worker) this.#spawn();
    const state = await this.#call('load', { modelUrl: scene.modelUrl || this.modelUrl.href });
    this.loaded = true;
    this.sceneRevision = scene.revision || 'phase1-vertical-slice-v1';
    this.robotId = scene.robotId || 'phase1_articulated_joint';
    this.lastObservation = state;
    return { sceneRevision: this.sceneRevision, robotId: this.robotId, observation: state };
  }

  async reset() {
    this.#assertLoaded();
    this.lastObservation = await this.#call('reset');
    return this.lastObservation;
  }

  async acceptCommand(envelope) {
    this.#assertLoaded();
    if (envelope?.sceneRevision && this.sceneRevision && envelope.sceneRevision !== this.sceneRevision) {
      throw new Error('Stale scene revision');
    }
    if (envelope?.robotId && this.robotId && envelope.robotId !== this.robotId) {
      throw new Error('Command robot does not match loaded scene');
    }
    this.lastObservation = await this.#call('command', envelope.command);
    return { status: 'accepted', commandId: envelope.commandId, observation: this.lastObservation };
  }

  async advanceSteps(stepCount = 1) {
    this.#assertLoaded();
    this.lastObservation = await this.#call('step', { count: stepCount });
    return this.lastObservation;
  }

  async getObservation() {
    this.#assertLoaded();
    this.lastObservation = await this.#call('observe');
    return this.lastObservation;
  }

  async getDiagnostics() {
    return {
      backend: 'browser-mujoco',
      loaded: this.loaded,
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      pendingRequests: this.pending.size,
      workerActive: Boolean(this.worker),
    };
  }

  async pause() {
    this.#assertLoaded();
    this.lastObservation = await this.#call('pause');
    return this.lastObservation;
  }

  async resume() {
    this.#assertLoaded();
    this.lastObservation = await this.#call('resume');
    return this.lastObservation;
  }

  async cancelRun() {
    if (!this.worker) return false;
    this.#terminate('cancelled');
    this.loaded = false;
    this.lastObservation = null;
    return true;
  }

  async exportTrace() {
    return {
      backend: 'browser-mujoco',
      phase: 'vertical-slice',
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      observation: this.loaded ? await this.getObservation() : this.lastObservation,
    };
  }

  async dispose() {
    if (this.disposed) return;
    this.disposed = true;
    if (this.worker) {
      try { await this.#call('dispose'); } catch {}
      this.#terminate('disposed');
    }
    this.loaded = false;
    this.lastObservation = null;
  }

  #spawn() {
    this.worker = new Worker(this.workerUrl, { type: 'module', name: 'robobuddy-mujoco-physics' });
    this.worker.onmessage = ({ data }) => {
      const request = this.pending.get(data?.id);
      if (!request) return;
      this.pending.delete(data.id);
      if (data.ok) request.resolve(data.payload);
      else request.reject(new Error(data.error || 'MuJoCo worker request failed'));
    };
    this.worker.onerror = (event) => this.#terminate(event.message || 'worker error');
  }

  #call(op, payload = {}) {
    if (!this.worker) throw new Error('MuJoCo worker is not running');
    const id = ++this.sequence;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.worker.postMessage({ id, op, payload });
    });
  }

  #terminate(reason) {
    this.worker?.terminate();
    this.worker = null;
    for (const { reject } of this.pending.values()) reject(new Error(`MuJoCo worker ${reason}`));
    this.pending.clear();
  }

  #assertLoaded() {
    if (!this.loaded) throw new Error('MuJoCo backend is not loaded');
    if (this.disposed) throw new Error('Backend is disposed');
  }
}

assertPhysicsBackend(new BrowserMuJoCoBackend());
