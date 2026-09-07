import { assertPhysicalBackend } from './backend-contract.js';

export class BrowserMuJoCoBackend {
  constructor({ workerUrl = new URL('./mujoco-worker.js', import.meta.url), modelUrl = new URL('../../models/vertical-slice/model.xml', import.meta.url) } = {}) {
    this.workerUrl = workerUrl;
    this.modelUrl = modelUrl;
    this.worker = null;
    this.sequence = 0;
    this.pending = new Map();
    this.loaded = false;
    this.disposed = false;
  }

  async load() {
    if (this.disposed) throw new Error('Backend is disposed');
    if (!this.worker) this.#spawn();
    const state = await this.#call('load', { modelUrl: this.modelUrl.href });
    this.loaded = true;
    return state;
  }

  async reset() {
    this.#assertLoaded();
    return this.#call('reset');
  }

  async command(command) {
    this.#assertLoaded();
    return this.#call('command', command);
  }

  async step(stepCount = 1) {
    this.#assertLoaded();
    return this.#call('step', { count: stepCount });
  }

  async observe() {
    this.#assertLoaded();
    return this.#call('observe');
  }

  async pause() {
    this.#assertLoaded();
    return this.#call('pause');
  }

  async resume() {
    this.#assertLoaded();
    return this.#call('resume');
  }

  async cancel() {
    if (!this.worker) return false;
    this.#terminate('cancelled');
    this.loaded = false;
    return true;
  }

  async exportTrace() {
    return { backend: 'browser-mujoco', phase: 'vertical-slice', observation: this.loaded ? await this.observe() : null };
  }

  async dispose() {
    if (this.disposed) return;
    this.disposed = true;
    if (this.worker) {
      try { await this.#call('dispose'); } catch {}
      this.#terminate('disposed');
    }
    this.loaded = false;
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

assertPhysicalBackend(new BrowserMuJoCoBackend());
