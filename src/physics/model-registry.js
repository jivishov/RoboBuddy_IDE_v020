export const MODEL_EVIDENCE = Object.freeze({
  MODEL_DERIVED: 'model-derived',
  NUMERICALLY_VERIFIED: 'numerically-verified',
  HARDWARE_COMPARED: 'hardware-compared',
});

export const EXECUTION_BACKENDS = Object.freeze({
  LEGACY: 'legacy',
  BROWSER_MUJOCO: 'browser-mujoco',
  NATIVE_MUJOCO: 'native-mujoco',
});

const registry = new Map();

export function registerModelPackage(manifest) {
  if (!manifest?.id || !manifest?.robotId || !manifest?.source?.url || !manifest?.source?.revision) {
    throw new TypeError('Model package requires id, robotId, source.url, and source.revision');
  }
  if (registry.has(manifest.id)) throw new Error(`Duplicate model package: ${manifest.id}`);
  const frozen = Object.freeze(structuredClone(manifest));
  registry.set(frozen.id, frozen);
  return frozen;
}

export function getModelPackage(id) {
  return registry.get(id) || null;
}

export function listModelPackages() {
  return [...registry.values()];
}

export function capabilityRecord({ backend, capability, evidence, limitations = [] }) {
  if (!Object.values(EXECUTION_BACKENDS).includes(backend)) throw new TypeError(`Unknown backend: ${backend}`);
  if (!Object.values(MODEL_EVIDENCE).includes(evidence)) throw new TypeError(`Unknown evidence label: ${evidence}`);
  return Object.freeze({ backend, capability, evidence, limitations: [...limitations] });
}
