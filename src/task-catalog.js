// Thin extension layer: the pre-existing catalog remains byte-for-byte in task-catalog-base.js.
export * from './task-catalog-base.js';
import * as base from './task-catalog-base.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from './physics/openarm-model-package.js';
import { OPENARM_LAB_BUILDER_SCENE_ID, OPENARM_LAB_BUILDER_SCENE_REVISION } from './physics/openarm-lab-builder.js';

export const OPENARM_LAB_BUILDER_SCENARIO = Object.freeze({
  schema: 'robobuddy.physical-workspace.v1',
  schemaVersion: 1,
  simulationMode: 'physical_mujoco',
  workspaceRevision: OPENARM_LAB_BUILDER_SCENE_REVISION,
  executionBudget: Object.freeze({ pythonWallTimeMs: 120000 }),
  id: 'openarm-image-assisted-lab-builder',
  title: 'Image-Assisted OpenArm Lab Builder',
  brief: 'Start from the source-derived OpenArm V2 bimanual system, its visible world-fixed mount/pedestal, and ground only. A multimodal agent may interpret a supplied laboratory image externally and use bounded WebMCP SceneSpec authoring to construct the bench, dry rigid-body equipment, receiver and obstacles. No reference-task bench, flask, beaker, rack, tray, instrument or hidden goal object is preloaded.',
  robotId: OPENARM_V2_PHASE5A_MODEL_PACKAGE.robotId,
  physicalSceneId: OPENARM_LAB_BUILDER_SCENE_ID,
  physicalSceneRevision: OPENARM_LAB_BUILDER_SCENE_REVISION,
  modelPackage: 'runtime-derived-openarm-lab-builder',
  modelId: 'runtime-derived-openarm-lab-builder',
  labBuilder: true,
  physicalApi: Object.freeze({ version: 'robobuddy.sim.v1', angleUnit: 'rad', timeUnit: 's', lengthUnit: 'm' }),
  canonicalModel: Object.freeze({
    repository: 'enactic/openarm_mujoco',
    revision: OPENARM_V2_PHASE5A_MODEL_PACKAGE.source.revision,
    sourcePath: 'v2/openarm_bimanual.xml',
    authority: 'MuJoCo PhysicsSession; robot mechanics come from the existing source-derived OpenArm package, not from the reference photograph',
  }),
  frames: Object.freeze({
    physics: 'MuJoCo right-handed Z-up world, metres/radians; source-derived OpenArm mount retained as a declared fixed boundary condition',
    rendering: 'Three.js Y-up millimetres from MuJoCo observations; presentation never advances physics',
  }),
  portablePython: Object.freeze({ referenceActions: Object.freeze([]) }),
  taskEvaluation: Object.freeze({
    source: 'Application-owned frozen dry-transfer evaluator over MuJoCo body state, velocities and named contacts',
    requires: Object.freeze(['new bilateral grasp', 'lift', 'carried displacement', 'receiver region and support', 'release', 'settling', 'retreat']),
    syntheticSuccessEvents: false,
  }),
  capabilities: Object.freeze({
    supported: Object.freeze(['structured image-assisted scene authoring', 'parameterized dry rigid-body bench/equipment', 'dry vial/block transfer', 'persistent editable bounded program', 'ground-truth simulation observations']),
    unavailable: Object.freeze(['camera-based execution/perception', 'general image-to-3D reconstruction', 'arbitrary articulated instruments', 'coordinated bimanual manipulation', 'laboratory process physics']),
  }),
  limitations: Object.freeze([
    'The authoring image is interpreted by the external multimodal agent; RoboBuddy does not require or claim its own vision model.',
    'Image-estimated dimensions and material/contact properties remain assumptions until independently measured. Tight-clearance claims must be treated as conditional on those inputs.',
    'The initial transfer planner is conservative task-space clearance planning plus runtime IK/physics; it is not a mathematical continuous-collision guarantee for every robot link.',
    'No object attachment, snapping, pose assignment for task credit, collision disabling, hidden reference arrangement, hardware transport or hardware validation is provided.',
  ]),
});

export const OPENARM_LAB_BUILDER_TASK = Object.freeze({
  profileId: 'openarm', id: OPENARM_LAB_BUILDER_SCENARIO.id, title: OPENARM_LAB_BUILDER_SCENARIO.title,
  robotId: OPENARM_LAB_BUILDER_SCENARIO.robotId, simulationMode: 'physical_mujoco',
  physicalSceneId: OPENARM_LAB_BUILDER_SCENE_ID, labBuilder: true,
});

export function tasksForProfile(profileId) {
  const tasks = base.tasksForProfile(profileId);
  return profileId === 'openarm' ? Object.freeze([...tasks, OPENARM_LAB_BUILDER_TASK]) : tasks;
}
export function defaultTaskId(profileId) { return tasksForProfile(profileId)[0]?.id || ''; }
export function taskDescriptor(profileId, taskId) { return tasksForProfile(profileId).find(item => item.id === taskId) || tasksForProfile(profileId)[0] || null; }
export async function loadPatchedScenario(profileId, taskId) {
  if (profileId === 'openarm' && taskId === OPENARM_LAB_BUILDER_TASK.id) return structuredClone(OPENARM_LAB_BUILDER_SCENARIO);
  return base.loadPatchedScenario(profileId, taskId);
}
export function taskPatchProvenance(descriptor) {
  if (descriptor?.id === OPENARM_LAB_BUILDER_TASK.id) return {
    repository: 'jivishov/RoboBuddy_IDE_v020',
    upstreamRepository: 'enactic/openarm_mujoco',
    upstreamRevision: OPENARM_V2_PHASE5A_MODEL_PACKAGE.source.revision,
    scenarioId: descriptor.id,
    physicalSceneId: OPENARM_LAB_BUILDER_SCENE_ID,
    modelPackage: 'runtime-derived-from-verified-openarm-base',
    simulationMode: 'physical_mujoco',
    standalonePlan: true,
  };
  return base.taskPatchProvenance(descriptor);
}
