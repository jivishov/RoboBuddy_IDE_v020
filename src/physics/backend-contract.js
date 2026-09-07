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
