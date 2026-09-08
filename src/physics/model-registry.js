export const MODEL_EVIDENCE = Object.freeze({
  MODEL_DERIVED: 'model-derived',
  NUMERICALLY_VERIFIED: 'numerically-verified',
  HARDWARE_COMPARED: 'hardware-compared',
});

export const PARAMETER_EVIDENCE = Object.freeze({
  SOURCE_DERIVED: 'source-derived',
  ESTIMATED: 'estimated',
  CALIBRATION_REQUIRED: 'calibration-required',
});

export const EXECUTION_BACKENDS = Object.freeze({
  LEGACY: 'legacy',
  BROWSER_MUJOCO: 'browser-mujoco',
  NATIVE_MUJOCO: 'native-mujoco',
});

const registry = new Map();
const MODEL_ASSET_RE = /^models\/[A-Za-z0-9._/-]+\.xml$/;
const SHA256_RE = /^[0-9a-f]{64}$/;

function stringArray(value, label) {
  if (!Array.isArray(value) || value.some((item) => typeof item !== 'string' || !item)) {
    throw new TypeError(`${label} must be an array of non-empty strings`);
  }
  if (new Set(value).size !== value.length) throw new TypeError(`${label} must contain unique values`);
  return value;
}

function validateDescriptorList(items, label, requiredKeys) {
  if (!Array.isArray(items) || items.length === 0) throw new TypeError(`${label} must be a non-empty array`);
  const ids = new Set();
  for (const [index, item] of items.entries()) {
    if (!item || typeof item !== 'object' || Array.isArray(item)) throw new TypeError(`${label}[${index}] must be an object`);
    for (const key of requiredKeys) {
      if (typeof item[key] !== 'string' || !item[key]) throw new TypeError(`${label}[${index}] requires ${key}`);
    }
    if (ids.has(item.id)) throw new TypeError(`${label} contains duplicate id ${item.id}`);
    ids.add(item.id);
  }
}

export function validateModelPackage(manifest) {
  if (!manifest || typeof manifest !== 'object' || Array.isArray(manifest)) throw new TypeError('Model package must be an object');
  if (!manifest.id || !manifest.robotId || !manifest.source?.url || !manifest.source?.revision) {
    throw new TypeError('Model package requires id, robotId, source.url, and source.revision');
  }
  if (!MODEL_ASSET_RE.test(manifest.asset || '') || String(manifest.asset).includes('..')) {
    throw new TypeError('Model package asset must be a repository-local models/*.xml path');
  }
  if (!SHA256_RE.test(String(manifest.sha256 || '').toLowerCase())) throw new TypeError('Model package requires a lowercase SHA-256');
  if (typeof manifest.license !== 'string' || !manifest.license) throw new TypeError('Model package requires license');
  if (!manifest.physics || !Number.isFinite(Number(manifest.physics.timestepSeconds)) || Number(manifest.physics.timestepSeconds) <= 0) {
    throw new TypeError('Model package requires a positive physics.timestepSeconds');
  }
  if (typeof manifest.physics.integrator !== 'string' || !manifest.physics.integrator) throw new TypeError('Model package requires physics.integrator');
  validateDescriptorList(manifest.joints, 'joints', ['id']);
  validateDescriptorList(manifest.actuators, 'actuators', ['id', 'jointId', 'controllerId']);
  const jointIds = new Set(manifest.joints.map((item) => item.id));
  for (const actuator of manifest.actuators) {
    if (!jointIds.has(actuator.jointId)) throw new TypeError(`Actuator ${actuator.id} maps unknown joint ${actuator.jointId}`);
  }
  if (manifest.bodies != null) validateDescriptorList(manifest.bodies, 'bodies', ['id']);
  stringArray(manifest.controllers, 'controllers');
  for (const actuator of manifest.actuators) {
    if (!manifest.controllers.includes(actuator.controllerId)) throw new TypeError(`Actuator ${actuator.id} uses undeclared controller ${actuator.controllerId}`);
  }
  if (manifest.sceneConstraints?.fixtures != null) stringArray(manifest.sceneConstraints.fixtures, 'sceneConstraints.fixtures');
  if (manifest.sceneConstraints?.objects != null) stringArray(manifest.sceneConstraints.objects, 'sceneConstraints.objects');
  return manifest;
}

export function registerModelPackage(manifest) {
  validateModelPackage(manifest);
  if (registry.has(manifest.id)) throw new Error(`Duplicate model package: ${manifest.id}`);
  const frozen = Object.freeze(structuredClone(manifest));
  registry.set(frozen.id, frozen);
  return frozen;
}

export function getModelPackage(id) {
  return registry.get(id) || null;
}

export function requireModelPackage(id) {
  const modelPackage = getModelPackage(id);
  if (!modelPackage) throw new Error(`Unknown physical model package: ${id}`);
  return modelPackage;
}

export function listModelPackages() {
  return [...registry.values()];
}

export function capabilityRecord({ backend, capability, evidence, limitations = [] }) {
  if (!Object.values(EXECUTION_BACKENDS).includes(backend)) throw new TypeError(`Unknown backend: ${backend}`);
  if (!Object.values(MODEL_EVIDENCE).includes(evidence)) throw new TypeError(`Unknown evidence label: ${evidence}`);
  return Object.freeze({ backend, capability, evidence, limitations: [...limitations] });
}
