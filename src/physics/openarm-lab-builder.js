export * from './openarm-lab-builder-base.js';
import {
  OPENARM_LAB_BUILDER_SCENE_ID,
  OPENARM_LAB_BUILDER_SCENE_REVISION,
  OPENARM_LAB_PROJECT_VERSION,
} from './openarm-lab-builder-base.js';
import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';

// The Lab Builder identity belongs to the app/scenario layer, not the shared physical-scene
// contract. Keep the scene descriptor strictly inside backend-contract.js so PhysicsSession can
// validate it exactly like every other MuJoCo workspace.
export function openArmLabBuilderScene(modelPackage) {
  if (typeof modelPackage !== 'string' || !modelPackage) throw new TypeError('Lab Builder scene requires a registered model package id');
  return Object.freeze({
    schemaVersion: PHYSICS_BACKEND_API_VERSION,
    id: OPENARM_LAB_BUILDER_SCENE_ID,
    revision: OPENARM_LAB_BUILDER_SCENE_REVISION,
    robotId: OPENARM_V2_PHASE5A_MODEL_PACKAGE.robotId,
    modelPackage,
    physics: Object.freeze({ ...OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics }),
    fixtures: Object.freeze([{ id: 'floor' }, { id: 'openarm_mount' }]),
    objects: Object.freeze([]),
    controllers: Object.freeze([...OPENARM_V2_PHASE5A_MODEL_PACKAGE.controllers]),
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
