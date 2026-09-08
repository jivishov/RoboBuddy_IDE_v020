export const PHYSICS_BACKEND_API_VERSION = '0.2.0-alpha.1';

export const PhysicsBackendState = Object.freeze({
  IDLE: 'idle',
  LOADING: 'loading',
  READY: 'ready',
  RUNNING: 'running',
  PAUSED: 'paused',
  FAILED: 'failed',
  DISPOSED: 'disposed',
});

const SCENE_KEYS = new Set([
  'schemaVersion', 'id', 'revision', 'robotId', 'modelPackage', 'legacyTaskId',
  'physics', 'fixtures', 'objects', 'controllers', 'taskGoal',
]);
const PHYSICS_KEYS = new Set(['timestepSeconds', 'integrator', 'iterations', 'lsIterations']);
const INTEGRATORS = new Set(['Euler', 'RK4', 'implicit', 'implicitfast']);

export function assertPhysicsBackend(backend) {
  const required = [
    'loadScene',
    'reset',
    'acceptCommand',
    'advanceSteps',
    'getObservation',
    'getDiagnostics',
    'pause',
    'resume',
    'cancelRun',
    'exportTrace',
    'dispose',
  ];
  const missing = required.filter((name) => typeof backend?.[name] !== 'function');
  if (missing.length) throw new TypeError(`Invalid physics backend: missing ${missing.join(', ')}`);
  return backend;
}

export function assertPhysicalScene(scene) {
  if (!scene || typeof scene !== 'object' || Array.isArray(scene)) throw new TypeError('Physical scene must be an object');
  for (const key of Object.keys(scene)) {
    if (!SCENE_KEYS.has(key)) throw new TypeError(`Unknown physical scene field: ${key}`);
  }
  for (const key of ['schemaVersion', 'id', 'revision', 'robotId', 'modelPackage']) {
    if (typeof scene[key] !== 'string' || !scene[key]) throw new TypeError(`Physical scene requires non-empty ${key}`);
  }
  if (scene.schemaVersion !== PHYSICS_BACKEND_API_VERSION) {
    throw new Error(`Unsupported physical scene schema ${scene.schemaVersion}`);
  }
  if (!scene.physics || typeof scene.physics !== 'object' || Array.isArray(scene.physics)) {
    throw new TypeError('Physical scene requires physics settings');
  }
  for (const key of Object.keys(scene.physics)) {
    if (!PHYSICS_KEYS.has(key)) throw new TypeError(`Unknown physics setting: ${key}`);
  }
  const timestep = Number(scene.physics.timestepSeconds);
  if (!Number.isFinite(timestep) || timestep <= 0) throw new RangeError('physics.timestepSeconds must be positive');
  if (!INTEGRATORS.has(scene.physics.integrator)) throw new TypeError(`Unsupported physics integrator: ${scene.physics.integrator}`);
  if (scene.physics.iterations != null && (!Number.isInteger(scene.physics.iterations) || scene.physics.iterations < 1)) {
    throw new RangeError('physics.iterations must be a positive integer when provided');
  }
  if (scene.physics.lsIterations != null && (!Number.isInteger(scene.physics.lsIterations) || scene.physics.lsIterations < 0)) {
    throw new RangeError('physics.lsIterations must be a non-negative integer when provided');
  }
  for (const key of ['fixtures', 'objects', 'controllers']) {
    if (scene[key] != null && !Array.isArray(scene[key])) throw new TypeError(`${key} must be an array when provided`);
  }
  if (Array.isArray(scene.controllers) && new Set(scene.controllers).size !== scene.controllers.length) {
    throw new TypeError('controllers must contain unique values');
  }
  return scene;
}

export function makeCommandEnvelope({ sessionId, epoch, commandId, sceneRevision, robotId, command, maxSteps = null }) {
  if (!sessionId || !commandId || !sceneRevision || !robotId || !command?.type) {
    throw new TypeError('Incomplete physics command envelope');
  }
  if (!Number.isInteger(epoch) || epoch < 1) throw new RangeError('Physics command epoch must be a positive integer');
  if (maxSteps != null && (!Number.isInteger(maxSteps) || maxSteps < 1)) {
    throw new RangeError('maxSteps must be null or a positive integer');
  }
  return Object.freeze({
    schemaVersion: PHYSICS_BACKEND_API_VERSION,
    sessionId: String(sessionId),
    epoch,
    commandId: String(commandId),
    sceneRevision: String(sceneRevision),
    robotId: String(robotId),
    command: structuredClone(command),
    maxSteps,
  });
}
