export * from './openarm-lab-builder-base.js';
import {
  OPENARM_LAB_BUILDER_SCENE_ID,
  OPENARM_LAB_BUILDER_SCENE_REVISION,
  OPENARM_LAB_PROJECT_VERSION,
} from './openarm-lab-builder-base.js';
import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';
import { requireModelPackage } from './model-registry.js';

// Lab Builder identity belongs to the app/scenario layer, not the shared physical-scene
// contract. Build a strict scene descriptor from the actual registered transient package so
// fixture/object identities cannot drift from the model that the worker will compile.
export function openArmLabBuilderScene(modelPackageId) {
  if (typeof modelPackageId !== 'string' || !modelPackageId) throw new TypeError('Lab Builder scene requires a registered model package id');
  const modelPackage = requireModelPackage(modelPackageId);
  const fixtures = (modelPackage.sceneConstraints?.fixtures || []).map(id => Object.freeze({ id }));
  const objects = (modelPackage.sceneConstraints?.objects || []).map(id => Object.freeze({ id }));
  return Object.freeze({
    schemaVersion: PHYSICS_BACKEND_API_VERSION,
    id: OPENARM_LAB_BUILDER_SCENE_ID,
    revision: OPENARM_LAB_BUILDER_SCENE_REVISION,
    robotId: modelPackage.robotId || OPENARM_V2_PHASE5A_MODEL_PACKAGE.robotId,
    modelPackage: modelPackageId,
    physics: Object.freeze({ ...modelPackage.physics }),
    fixtures: Object.freeze(fixtures),
    objects: Object.freeze(objects),
    controllers: Object.freeze([...(modelPackage.controllers || [])]),
    taskGoal: null,
  });
}

// Portable projects omit unavailable optional sections instead of serializing null values.
// This keeps a freshly exported blank project valid input to the bounded reopen schema and never
// implies that a task or program exists when it has not yet been authored.
export function createLabProject({ sceneSpec = null, taskSpec = null, program = null, executionProfile = null } = {}) {
  return {
    schema_version: OPENARM_LAB_PROJECT_VERSION,
    scene: sceneSpec ? structuredClone(sceneSpec) : null,
    ...(taskSpec ? { task: structuredClone(taskSpec) } : {}),
    ...(program ? { program: structuredClone(program) } : {}),
    ...(executionProfile ? { execution_profile: structuredClone(executionProfile) } : {}),
    auto_start: false,
  };
}
