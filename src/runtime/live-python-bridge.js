const DEFAULT_TIMEOUT_MS = 15000;
const STEP_ALIGNMENT_TOLERANCE_SECONDS = 1e-9;

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

  async advance(seconds) {
    this.#assertLive();
    const duration = Number(seconds);
    if (!Number.isFinite(duration) || duration < 0) throw new RangeError('advance duration must be a finite non-negative number');
    const diagnostics = await this.#withTimeout(this.session.getDiagnostics());
    const timestep = Number(diagnostics?.timestepSeconds);
    if (!Number.isFinite(timestep) || timestep <= 0) throw new Error('Physical session did not report a valid MuJoCo timestep');
    if (duration === 0) return this.#withTimeout(this.session.getObservation({ view: 'ground_truth' }));
    const ratio = duration / timestep;
    const steps = Math.round(ratio);
    if (steps < 1 || Math.abs(duration - steps * timestep) > STEP_ALIGNMENT_TOLERANCE_SECONDS) {
      throw new RangeError(`advance duration ${duration}s must be an integer number of ${timestep}s physics steps`);
    }
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
