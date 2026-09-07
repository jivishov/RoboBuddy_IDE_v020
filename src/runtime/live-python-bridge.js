const DEFAULT_TIMEOUT_MS = 15000;

export class LivePythonBridge {
  constructor(session, { timeoutMs = DEFAULT_TIMEOUT_MS } = {}) {
    this.session = session;
    this.timeoutMs = timeoutMs;
    this.disposed = false;
  }

  async sendAction(action, options = {}) {
    this.#assertLive();
    return this.#withTimeout(this.session.sendCommand({ type: 'joint_action', action: structuredClone(action) }, options));
  }

  async advance(seconds, { controllerHz = 50 } = {}) {
    this.#assertLive();
    const duration = Number(seconds);
    if (!Number.isFinite(duration) || duration < 0) throw new RangeError('advance duration must be a finite non-negative number');
    const steps = Math.max(1, Math.round(duration * controllerHz));
    return this.#withTimeout(this.session.advanceSteps(steps));
  }

  async getObservation({ view = 'sensor' } = {}) {
    this.#assertLive();
    if (!['sensor', 'ground_truth'].includes(view)) throw new TypeError(`Unknown observation view: ${view}`);
    return this.#withTimeout(this.session.getObservation({ view }));
  }

  async cancel(reason = 'python-cancelled') {
    if (this.disposed) return false;
    return this.session.cancelRun(reason);
  }

  dispose() {
    this.disposed = true;
  }

  async #withTimeout(promise) {
    let timer;
    try {
      return await Promise.race([
        promise,
        new Promise((_, reject) => {
          timer = setTimeout(() => reject(new Error('Live simulation operation timed out')), this.timeoutMs);
        }),
      ]);
    } finally {
      clearTimeout(timer);
    }
  }

  #assertLive() {
    if (this.disposed) throw new Error('Live Python bridge is disposed');
  }
}
