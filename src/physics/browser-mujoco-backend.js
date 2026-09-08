import { assertPhysicalScene, assertPhysicsBackend, PHYSICS_BACKEND_API_VERSION, PhysicsBackendState } from './backend-contract.js';
import { PHASE1_SCENE } from './phase1-scene.js';

const MAX_TRACE_EVENTS = 512;
const MAX_STEP_BATCH = 100000;
const STEP_TIME_TOLERANCE_SECONDS = 1e-9;
const WORLD_FRAME = Object.freeze({
  id: 'mujoco_world',
  handedness: 'right-handed',
  upAxis: '+Z',
  linearUnit: 'm',
  angularUnit: 'rad',
});

function exactStringArray(actual, expected) {
  return Array.isArray(actual)
    && actual.length === expected.length
    && actual.every((value, index) => value === expected[index]);
}

function objectIds(items) {
  return Array.isArray(items) ? items.map((item) => item?.id) : [];
}

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
    this.lastScene = null;
    this.sessionId = null;
    this.epoch = 0;
    this.state = PhysicsBackendState.IDLE;
    this.trace = [];
    this.generation = 0;
    this.commandBudget = null;
  }

  async loadScene(scene = {}, context = {}) {
    this.#assertNotDisposed();
    assertPhysicalScene(scene);
    this.#assertPhase1Scene(scene);
    this.#validateNewEpoch(context);
    if (!this.worker) this.#spawn();
    this.state = PhysicsBackendState.LOADING;
    this.sessionId = String(context.sessionId);
    this.epoch = context.epoch;
    this.commandBudget = null;
    const generation = ++this.generation;
    try {
      const raw = await this.#call('load', { modelUrl: this.modelUrl.href });
      this.#assertGeneration(generation, context);
      this.loaded = true;
      this.sceneRevision = String(scene.revision);
      this.robotId = String(scene.robotId);
      const observation = this.#decorateObservation(raw);
      this.lastScene = {
        schemaVersion: scene.schemaVersion,
        id: scene.id,
        revision: this.sceneRevision,
        robotId: this.robotId,
        modelPackage: scene.modelPackage,
        physics: structuredClone(scene.physics),
        controllers: structuredClone(scene.controllers || []),
        model: structuredClone(observation.model),
      };
      this.lastObservation = observation;
      this.state = PhysicsBackendState.READY;
      this.#record('loadScene', { observation });
      return { sceneRevision: this.sceneRevision, robotId: this.robotId, observation };
    } catch (error) {
      if (generation === this.generation) this.state = PhysicsBackendState.FAILED;
      throw error;
    }
  }

  async reset(context = {}) {
    this.#assertLoaded();
    this.#adoptNewEpoch(context);
    this.commandBudget = null;
    const generation = ++this.generation;
    const raw = await this.#call('reset');
    this.#assertGeneration(generation, context);
    const observation = this.#decorateObservation(raw);
    this.lastObservation = observation;
    this.state = PhysicsBackendState.READY;
    this.#record('reset', { observation });
    return observation;
  }

  async acceptCommand(envelope) {
    this.#assertLoaded();
    if (envelope?.schemaVersion !== PHYSICS_BACKEND_API_VERSION) throw new Error('Unsupported physics command schema');
    this.#assertContext(envelope);
    if (envelope.sceneRevision !== this.sceneRevision) throw new Error('Stale scene revision');
    if (envelope.robotId !== this.robotId) throw new Error('Command robot does not match loaded scene');
    if (envelope.command?.type === 'set_joint_target' && !Number.isInteger(envelope.maxSteps)) {
      throw new RangeError('Phase 1 set_joint_target requires a positive maxSteps budget');
    }
    const wasPaused = this.state === PhysicsBackendState.PAUSED;
    const generation = this.generation;
    const raw = await this.#call('command', envelope.command);
    this.#assertGeneration(generation, envelope);
    const observation = this.#decorateObservation(raw);
    this.lastObservation = observation;
    this.commandBudget = Number.isInteger(envelope.maxSteps)
      ? { commandId: envelope.commandId, remainingSteps: envelope.maxSteps }
      : null;
    this.state = wasPaused ? PhysicsBackendState.PAUSED : PhysicsBackendState.READY;
    this.#record('command', {
      commandId: envelope.commandId,
      command: structuredClone(envelope.command),
      maxSteps: envelope.maxSteps,
      remainingSteps: this.commandBudget?.remainingSteps ?? null,
      observation,
    });
    return { status: 'accepted', commandId: envelope.commandId, remainingSteps: this.commandBudget?.remainingSteps ?? null, observation };
  }

  async advanceSteps(stepCount = 1, context = {}) {
    this.#assertLoaded();
    this.#assertContext(context);
    if (!Number.isInteger(stepCount) || stepCount < 1 || stepCount > MAX_STEP_BATCH) {
      throw new RangeError(`stepCount must be an integer from 1 to ${MAX_STEP_BATCH}`);
    }
    if (this.commandBudget && stepCount > this.commandBudget.remainingSteps) {
      throw new RangeError(`Step request ${stepCount} exceeds remaining command budget ${this.commandBudget.remainingSteps}`);
    }
    const generation = this.generation;
    const previousTime = Number(this.lastObservation?.simulationTimeSeconds ?? 0);
    const wasPaused = this.state === PhysicsBackendState.PAUSED;
    if (!wasPaused) this.state = PhysicsBackendState.RUNNING;
    try {
      const raw = await this.#call('step', { count: stepCount });
      this.#assertGeneration(generation, context);
      const observation = this.#decorateObservation(raw);
      const executedSteps = this.#executedSteps(previousTime, observation.simulationTimeSeconds, observation.engine.timestepSeconds);
      if (wasPaused && executedSteps !== 0) throw new Error(`Paused MuJoCo advanced ${executedSteps} steps`);
      if (!wasPaused && executedSteps !== stepCount) throw new Error(`MuJoCo advanced ${executedSteps} steps for a request of ${stepCount}`);
      this.lastObservation = observation;
      if (this.commandBudget) this.commandBudget.remainingSteps -= executedSteps;
      this.state = wasPaused ? PhysicsBackendState.PAUSED : PhysicsBackendState.READY;
      this.#record('advanceSteps', {
        requestedSteps: stepCount,
        executedSteps,
        commandId: this.commandBudget?.commandId ?? null,
        remainingSteps: this.commandBudget?.remainingSteps ?? null,
        observation,
      });
      return observation;
    } catch (error) {
      if (generation === this.generation && this.loaded) this.state = PhysicsBackendState.FAILED;
      throw error;
    }
  }

  async getObservation(context = {}) {
    this.#assertLoaded();
    this.#assertContext(context);
    const requestedView = context.view ?? 'ground_truth';
    if (requestedView !== 'ground_truth') throw new Error(`Observation view ${requestedView} is unsupported by the Phase 1 slice`);
    const generation = this.generation;
    const raw = await this.#call('observe');
    this.#assertGeneration(generation, context);
    const observation = this.#decorateObservation(raw, requestedView);
    this.lastObservation = observation;
    return observation;
  }

  async getDiagnostics(context = {}) {
    if (this.sessionId && context?.sessionId) this.#assertSession(context);
    return {
      backend: 'browser-mujoco',
      state: this.state,
      loaded: this.loaded,
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      sessionId: this.sessionId,
      epoch: this.epoch,
      pendingRequests: this.pending.size,
      workerActive: Boolean(this.worker),
      model: this.lastObservation?.model ? structuredClone(this.lastObservation.model) : null,
      engineVersion: this.lastObservation?.engine?.version || null,
      engineVersionEvidence: this.lastObservation?.engine?.versionEvidence || null,
      timestepSeconds: this.lastObservation?.engine?.timestepSeconds ?? null,
      worldFrame: structuredClone(WORLD_FRAME),
      activeCommandId: this.commandBudget?.commandId ?? null,
      remainingCommandSteps: this.commandBudget?.remainingSteps ?? null,
      traceEvents: this.trace.length,
    };
  }

  async pause(context = {}) {
    this.#assertLoaded();
    this.#assertContext(context);
    const generation = this.generation;
    const raw = await this.#call('pause');
    this.#assertGeneration(generation, context);
    const observation = this.#decorateObservation(raw);
    this.lastObservation = observation;
    this.state = PhysicsBackendState.PAUSED;
    this.#record('pause', { observation });
    return observation;
  }

  async resume(context = {}) {
    this.#assertLoaded();
    this.#assertContext(context);
    const generation = this.generation;
    const raw = await this.#call('resume');
    this.#assertGeneration(generation, context);
    const observation = this.#decorateObservation(raw);
    this.lastObservation = observation;
    this.state = PhysicsBackendState.READY;
    this.#record('resume', { observation });
    return observation;
  }

  async cancelRun(context = {}) {
    this.#assertLoaded();
    this.#adoptNewEpoch(context);
    ++this.generation;
    this.#record('cancelRun', { reason: context.reason || 'cancelled' });
    this.#terminate('cancelled');
    this.loaded = false;
    this.state = PhysicsBackendState.IDLE;
    this.sceneRevision = null;
    this.robotId = null;
    this.lastScene = null;
    this.lastObservation = null;
    this.commandBudget = null;
    return { cancelled: true, reloadRequired: true };
  }

  async exportTrace(context = {}) {
    if (this.sessionId && context?.sessionId) this.#assertSession(context);
    return {
      backend: 'browser-mujoco',
      phase: 'vertical-slice',
      apiVersion: PHYSICS_BACKEND_API_VERSION,
      scene: this.lastScene ? structuredClone(this.lastScene) : null,
      sessionId: this.sessionId,
      epoch: this.epoch,
      events: structuredClone(this.trace),
      lastObservation: structuredClone(this.lastObservation),
    };
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    ++this.generation;
    this.#terminate('disposed');
    this.loaded = false;
    this.state = PhysicsBackendState.DISPOSED;
    this.lastObservation = null;
    this.commandBudget = null;
  }

  #decorateObservation(raw = {}, view = 'ground_truth') {
    const simulationTimeSeconds = Number(raw.simulationTime);
    const timestepSeconds = Number(raw.engine?.timestepSeconds);
    const modelSha256 = String(raw.model?.sha256 || '').toLowerCase();
    if (!Number.isFinite(simulationTimeSeconds) || simulationTimeSeconds < 0) throw new Error('MuJoCo returned invalid simulation time');
    if (!Number.isFinite(timestepSeconds) || timestepSeconds <= 0) throw new Error('MuJoCo returned invalid timestep');
    if (!/^[0-9a-f]{64}$/.test(modelSha256)) throw new Error('MuJoCo worker did not provide a valid model SHA-256');
    return {
      schemaVersion: PHYSICS_BACKEND_API_VERSION,
      sessionId: this.sessionId,
      epoch: this.epoch,
      simulationTimeSeconds,
      view,
      robotId: this.robotId,
      frames: { world: structuredClone(WORLD_FRAME) },
      model: {
        id: String(raw.model?.id || ''),
        asset: String(raw.model?.asset || ''),
        sha256: modelSha256,
      },
      engine: {
        name: 'MuJoCo',
        version: raw.engine?.version == null ? null : String(raw.engine.version),
        versionEvidence: String(raw.engine?.versionEvidence || 'unknown'),
        timestepSeconds,
      },
      joints: structuredClone(raw.joints || {}),
      bodies: structuredClone(raw.bodies || {}),
      contactCount: Math.max(0, Number(raw.contactCount) || 0),
      contactsReadable: Boolean(raw.contactsReadable),
      contacts: structuredClone(Array.isArray(raw.contacts) ? raw.contacts : []),
      sensors: {},
    };
  }

  #executedSteps(previousTime, nextTime, timestep) {
    const delta = nextTime - previousTime;
    if (!Number.isFinite(delta) || delta < -STEP_TIME_TOLERANCE_SECONDS) throw new Error('MuJoCo simulation time moved backwards');
    if (Math.abs(delta) <= STEP_TIME_TOLERANCE_SECONDS) return 0;
    const ratio = delta / timestep;
    const steps = Math.round(ratio);
    if (steps < 0 || Math.abs(delta - steps * timestep) > STEP_TIME_TOLERANCE_SECONDS) {
      throw new Error(`MuJoCo simulation-time delta ${delta} is not an integer number of ${timestep}s steps`);
    }
    return steps;
  }

  #assertPhase1Scene(scene) {
    if (scene.id !== PHASE1_SCENE.id) throw new Error(`Phase 1 scene id must be ${PHASE1_SCENE.id}`);
    if (scene.revision !== PHASE1_SCENE.revision) throw new Error(`Phase 1 scene revision must be ${PHASE1_SCENE.revision}`);
    if (scene.robotId !== PHASE1_SCENE.robotId) throw new Error(`Phase 1 robotId must be ${PHASE1_SCENE.robotId}`);
    if (scene.modelPackage !== PHASE1_SCENE.modelPackage) throw new Error(`Phase 1 modelPackage must be ${PHASE1_SCENE.modelPackage}`);
    if (Math.abs(Number(scene.physics.timestepSeconds) - PHASE1_SCENE.physics.timestepSeconds) > 1e-12) {
      throw new Error(`Phase 1 timestep must be ${PHASE1_SCENE.physics.timestepSeconds}`);
    }
    if (scene.physics.integrator !== PHASE1_SCENE.physics.integrator) throw new Error(`Phase 1 integrator must be ${PHASE1_SCENE.physics.integrator}`);
    if (!exactStringArray(scene.controllers, PHASE1_SCENE.controllers)) throw new Error('Phase 1 controller set does not match the pinned scene');
    if (!exactStringArray(objectIds(scene.fixtures), objectIds(PHASE1_SCENE.fixtures))) throw new Error('Phase 1 fixture identities do not match the pinned scene');
    if (!exactStringArray(objectIds(scene.objects), objectIds(PHASE1_SCENE.objects))) throw new Error('Phase 1 object identities do not match the pinned scene');
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
    this.worker.onerror = (event) => {
      if (!this.disposed) this.state = PhysicsBackendState.FAILED;
      this.#terminate(event.message || 'worker error');
    };
  }

  #call(op, payload = {}) {
    if (!this.worker) throw new Error('MuJoCo worker is not running');
    const id = ++this.sequence;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.worker.postMessage({ id, op, payload });
    });
  }

  #record(type, details = {}) {
    this.trace.push({
      type,
      sessionId: this.sessionId,
      epoch: this.epoch,
      sceneRevision: this.sceneRevision,
      robotId: this.robotId,
      simulationTimeSeconds: details.observation?.simulationTimeSeconds ?? this.lastObservation?.simulationTimeSeconds ?? null,
      ...details,
    });
    if (this.trace.length > MAX_TRACE_EVENTS) this.trace.splice(0, this.trace.length - MAX_TRACE_EVENTS);
  }

  #validateNewEpoch(context) {
    if (!context?.sessionId || !Number.isInteger(context?.epoch) || context.epoch < 1) {
      throw new TypeError('Physics context requires sessionId and positive integer epoch');
    }
    if (this.sessionId && String(context.sessionId) !== this.sessionId) throw new Error('Backend is already owned by another physics session');
    if (context.epoch <= this.epoch) throw new Error('Stale physics epoch');
  }

  #adoptNewEpoch(context) {
    this.#assertSession(context);
    if (!Number.isInteger(context.epoch) || context.epoch <= this.epoch) throw new Error('Stale physics epoch');
    this.epoch = context.epoch;
  }

  #assertSession(context) {
    if (!context?.sessionId || String(context.sessionId) !== this.sessionId) throw new Error('Physics session mismatch');
  }

  #assertContext(context) {
    this.#assertSession(context);
    if (!Number.isInteger(context.epoch) || context.epoch !== this.epoch) throw new Error('Stale physics epoch');
  }

  #assertGeneration(generation, context) {
    if (generation !== this.generation) throw new Error('Stale physics response');
    this.#assertContext(context);
  }

  #terminate(reason) {
    this.worker?.terminate();
    this.worker = null;
    for (const { reject } of this.pending.values()) reject(new Error(`MuJoCo worker ${reason}`));
    this.pending.clear();
  }

  #assertLoaded() {
    this.#assertNotDisposed();
    if (!this.loaded || !this.worker) throw new Error('MuJoCo backend is not loaded');
  }

  #assertNotDisposed() {
    if (this.disposed) throw new Error('Backend is disposed');
  }
}

assertPhysicsBackend(new BrowserMuJoCoBackend());
