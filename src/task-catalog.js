export const TASK_PATCH_REVISION = '75fe2669c0ab0b029986de424c69162071174df8';
export const TASK_PATCH_SOURCE = 'jivishov/RoboBuddy_AI';

const ROOT = `https://cdn.jsdelivr.net/gh/${TASK_PATCH_SOURCE}@${TASK_PATCH_REVISION}/missions/lab-assistant/v2/definitions`;
export const UNITREE_G1_VISUAL_REVISION = '66d18a029a0caeb6a6075e681dbd9ecd6b22affa';

const task = (profileId, family, file, id, title, robotId) => Object.freeze({
  profileId, family, file, id, title, robotId,
  url: `${ROOT}/${family}/${file}`,
});

export const PATCH_TASKS = Object.freeze({
  openarm: Object.freeze([
    task('openarm', 'openarm', 'openarm-04-filtration-workcell.json', 'openarm-04-filtration-workcell', 'Bimanual Heater and Ring-Stand Stack', 'openarm_v2_bimanual'),
  ]),
  // These legacy SO-101 definitions remain read-only provenance/regression fixtures.
  // They are deliberately not returned by tasksForProfile('so101') in the physical version.
  so101: Object.freeze([
    task('so101', 'so101', 'so101-v2-06-quantitative-transfer.json', 'so101-v2-06-quantitative-transfer', 'Measured Two-Bottle Transfer Workcell', 'so101_follower'),
    task('so101', 'so101', 'so101-v2-08-burette-initial-reading.json', 'so101-v2-08-burette-initial-reading', 'Burette Receiver Clearance Calibration', 'so101_follower'),
    task('so101', 'so101', 'so101-v2-09-vacuum-filtration.json', 'so101-v2-09-vacuum-filtration', 'Vacuum Workcell Keep-Clear Preflight', 'so101_follower'),
  ]),
  lekiwi: Object.freeze([
    task('lekiwi', 'lekiwi', 'lekiwi-01-beaker-courier.json', 'lekiwi-01-beaker-courier', 'Beaker Courier', 'lekiwi_sim'),
  ]),
});

export const SO101_LEGACY_TASK_CLASSIFICATION = Object.freeze({
  'so101-v2-06-quantitative-transfer': Object.freeze({
    classification: 'B_DEFERRED',
    reason: 'Its current dry two-bottle staging redesign is rigid-body defensible, but faithful migration requires validated bottle collision geometry, two-object grasp/release trajectories, and workcell-specific controller evidence. Unsupported liquid transfer remains excluded.',
  }),
  'so101-v2-08-burette-initial-reading': Object.freeze({
    classification: 'B_DEFERRED',
    reason: 'Its dry bottle/beaker clearance setup is rigid-body defensible, but faithful migration requires validated glassware/burette fixture geometry and multi-object manipulation. Burette filling, dispensing, meniscus reading, and liquid metrology remain unsupported.',
  }),
  'so101-v2-09-vacuum-filtration': Object.freeze({
    classification: 'B_DEFERRED',
    reason: 'Its dry keep-clear preflight is rigid-body defensible, but faithful migration requires validated bottle/beaker/fixture geometry and corridor-aware multi-object manipulation. Vacuum, hose coupling, pressure, filtration, and liquids remain unsupported.',
  }),
});

const SO101_PHYSICAL_SCENARIO = Object.freeze({
  schema: 'robobuddy.physical-workspace.v1',
  schemaVersion: 1,
  simulationMode: 'physical_mujoco',
  workspaceRevision: 'so101-physical-block-transfer-v1',
  id: 'so101-physical-block-transfer',
  title: 'SO-101 Physical Block Transfer',
  brief: 'A controlled rigid-body benchmark workcell. Program the SO-101 to make real gripper contact with a free block, lift and carry it through MuJoCo dynamics, release it over the target, and leave it settled inside the target. This is simulator validation, not hardware calibration or a laboratory liquid-handling task.',
  robotId: 'so101_follower',
  physicalSceneId: 'p4-so101-benchmark-transfer',
  modelPackage: 'so101-manipulation-menagerie-8161bba-v1',
  modelId: 'robobuddy-so101-manipulation-v1',
  physicalApi: Object.freeze({ version: 'robobuddy.sim.v1', angleUnit: 'rad', timeUnit: 's', lengthUnit: 'm' }),
  canonicalModel: Object.freeze({
    repository: 'jivishov/RoboBuddy_AI',
    revision: UNITREE_G1_VISUAL_REVISION,
    module: 'simulator/js/robot-mesh-data-so101.js',
    sourceModel: 'SO101 official URDF baked by RoboBuddy_AI',
    authority: 'presentation-only; MuJoCo PhysicsSession is authoritative',
  }),
  frames: Object.freeze({
    physics: 'MuJoCo right-handed Z-up world, metres/radians',
    rendering: 'Three.js Y-up millimetres; radians are converted to degrees only at the canonical-rig rendering boundary',
  }),
  portablePython: Object.freeze({
    referenceActions: Object.freeze([
      Object.freeze({ label: 'Settle the free block under gravity/contact', hold_seconds: 0.20, targetsRad: Object.freeze({}) }),
      Object.freeze({ label: 'Approach the block', hold_seconds: 0.60, targetsRad: Object.freeze({ shoulder_pan: 0.005, shoulder_lift: 0.0 }) }),
      Object.freeze({ label: 'Close into real gripper contact', hold_seconds: 0.50, targetsRad: Object.freeze({ gripper: -0.04 }) }),
      Object.freeze({ label: 'Lift the contacted block', hold_seconds: 0.80, targetsRad: Object.freeze({ shoulder_lift: -0.35 }) }),
      Object.freeze({ label: 'Carry the block horizontally', hold_seconds: 0.90, targetsRad: Object.freeze({ shoulder_pan: 0.45 }) }),
      Object.freeze({ label: 'Lower over the target support', hold_seconds: 0.80, targetsRad: Object.freeze({ shoulder_lift: 0.0 }) }),
      Object.freeze({ label: 'Release the free block', hold_seconds: 0.50, targetsRad: Object.freeze({ gripper: 0.60 }) }),
      Object.freeze({ label: 'Allow gravity/contact settling', hold_seconds: 0.80, targetsRad: Object.freeze({}) }),
    ]),
  }),
  taskEvaluation: Object.freeze({
    source: 'MuJoCo observations and named geometry contacts',
    requires: Object.freeze(['gripper contact', 'lift', 'carried contact', 'target placement', 'release', 'gravity/contact settling', 'final rest inside target']),
    syntheticSuccessEvents: false,
  }),
  limitations: Object.freeze([
    'Synthetic benchmark block/support/target dimensions and contact parameters are declared simulator parameters, not measured laboratory geometry.',
    'SO-101 kinematics and retained camera/gripper collision geometry are source-derived from the pinned Menagerie adaptation; actuator/controller parameters are simulation estimates, not calibrated installed-servo measurements.',
    'No liquid, meniscus, vacuum, suction, force-sensor, tactile-sensor, or hardware-control capability is implied.',
  ]),
});

export const SO101_PHYSICAL_TASKS = Object.freeze([
  Object.freeze({
    profileId: 'so101',
    id: SO101_PHYSICAL_SCENARIO.id,
    title: SO101_PHYSICAL_SCENARIO.title,
    robotId: SO101_PHYSICAL_SCENARIO.robotId,
    simulationMode: SO101_PHYSICAL_SCENARIO.simulationMode,
    physicalSceneId: SO101_PHYSICAL_SCENARIO.physicalSceneId,
  }),
]);

const UNITREE_G1_RIG_SCENARIO = Object.freeze({
  schema: 'robobuddy.ide-rig-inspection.v1',
  simulationMode: 'kinematic_pose',
  workspaceRevision: 'unitree-g1-rig-v1',
  id: 'unitree-g1-kinematic-pose-inspection',
  title: 'Unitree G1 29-DoF Kinematic Pose Inspection',
  brief: 'Inspect the canonical Unitree G1 mesh through bounded named joint poses. This workspace deliberately has no collision/contact plant, gait, balance, or hardware-control claim.',
  robotId: 'unitree_g1_29dof',
  canonicalModel: Object.freeze({
    repository: 'jivishov/RoboBuddy_AI',
    revision: UNITREE_G1_VISUAL_REVISION,
    module: 'simulator/js/robot-mesh-data-unitree-g1.js',
    sourceRepository: 'unitreerobotics/unitree_ros',
    sourceRevision: 'dd4fa6866e523ad61324f658d63736e4eda3a6e4',
    sourcePath: 'robots/g1_description/g1_29dof.urdf',
    license: 'BSD-3-Clause',
  }),
  frames: Object.freeze({
    geometry: 'Three.js Y-up metres',
    transforms: 'Three.js Y-up millimetres',
    root: 'fixed visual root; no locomotion model',
  }),
  portablePython: Object.freeze({
    referenceActions: Object.freeze([
      Object.freeze({
        label: 'Upper-body joint-pose inspection',
        hold_seconds: 0.35,
        action: Object.freeze({
          waist_pitch_joint: 8,
          left_shoulder_pitch_joint: -35, left_shoulder_roll_joint: 28, left_elbow_joint: 45, left_wrist_pitch_joint: -10,
          right_shoulder_pitch_joint: -35, right_shoulder_roll_joint: -28, right_elbow_joint: 45, right_wrist_pitch_joint: -10,
        }),
      }),
      Object.freeze({
        label: 'Lower-body joint-pose inspection (root fixed)',
        hold_seconds: 0.35,
        action: Object.freeze({
          left_hip_roll_joint: 8, left_knee_joint: 22, left_ankle_pitch_joint: -10,
          right_hip_roll_joint: -8, right_knee_joint: 22, right_ankle_pitch_joint: -10,
        }),
      }),
      Object.freeze({
        label: 'Return inspected joints to neutral',
        hold_seconds: 0.35,
        action: Object.freeze({
          waist_pitch_joint: 0,
          left_shoulder_pitch_joint: 0, left_shoulder_roll_joint: 0, left_elbow_joint: 0, left_wrist_pitch_joint: 0,
          right_shoulder_pitch_joint: 0, right_shoulder_roll_joint: 0, right_elbow_joint: 0, right_wrist_pitch_joint: 0,
          left_hip_roll_joint: 0, left_knee_joint: 0, left_ankle_pitch_joint: 0,
          right_hip_roll_joint: 0, right_knee_joint: 0, right_ankle_pitch_joint: 0,
        }),
      }),
    ]),
  }),
});

export const UNITREE_G1_RIG_TASKS = Object.freeze([
  Object.freeze({
    profileId: 'unitree',
    id: UNITREE_G1_RIG_SCENARIO.id,
    title: UNITREE_G1_RIG_SCENARIO.title,
    robotId: UNITREE_G1_RIG_SCENARIO.robotId,
    simulationMode: 'kinematic_pose',
    source: `RoboBuddy_AI@${UNITREE_G1_VISUAL_REVISION}/simulator/js/robot-mesh-data-unitree-g1.js`,
  }),
]);

const MICRODUCK_SCENARIO = Object.freeze({
  schema: 'robobuddy.microduck-workspace.v1', simulationMode: 'policy_sim', workspaceRevision: 'microduck-cycle04-live-python-v1',
  id: 'microduck-policy-demonstrator', title: 'MicroDuck Articulated Policy Demonstrator', robotId: 'microduck_runtime_visual', variant: 'walking',
  brief: 'Run live async Python against the exact pinned policies and approximate browser dynamics while inspecting the official compact runtime visual and modeled state.',
  canonicalModel: Object.freeze({ repository: 'pollen-robotics/microduck', revision: '590b986bd8c0d50ae02cb3ea2f59c463b6828168', sourcePath: 'robotctl/assets/duck.bin', hierarchySourcePath: 'kinematics/assets/alpha/robot_walk.xml', geometry: 'official compact robotctl monitor mesh' }),
  frames: Object.freeze({ geometry: 'Source Z-up metres converted to Three.js Y-up and displayed at millimetre scale', hierarchy: 'pinned runtime XML and DUCK v1 body records', mouthRollersContacts: 'original configured approximations' }),
  portablePython: Object.freeze({ referenceActions: Object.freeze([]) }),
});
const MICRODUCK_TASKS = Object.freeze([Object.freeze({ profileId:'microduck', id:MICRODUCK_SCENARIO.id, title:MICRODUCK_SCENARIO.title, robotId:MICRODUCK_SCENARIO.robotId, simulationMode:'policy_sim' })]);

const cache = new Map();

export function tasksForProfile(profileId) {
  if (profileId === 'so101') return SO101_PHYSICAL_TASKS;
  if (profileId === 'unitree') return UNITREE_G1_RIG_TASKS;
  if (profileId === 'microduck') return MICRODUCK_TASKS;
  return PATCH_TASKS[profileId] || [];
}

export function defaultTaskId(profileId) {
  return tasksForProfile(profileId)[0]?.id || '';
}

export function taskDescriptor(profileId, taskId) {
  return tasksForProfile(profileId).find((item) => item.id === taskId) || tasksForProfile(profileId)[0] || null;
}

export async function loadPatchedScenario(profileId, taskId) {
  // Hidden legacy SO-101 definitions remain loadable by exact ID for regression/provenance
  // checks, but taskDescriptor()/tasksForProfile() never expose them in the physical UI.
  const legacySo101 = profileId === 'so101' ? PATCH_TASKS.so101.find((item) => item.id === taskId) : null;
  const descriptor = legacySo101 || taskDescriptor(profileId, taskId);
  if (!descriptor) return null;
  if (descriptor.simulationMode === 'physical_mujoco') return structuredClone(SO101_PHYSICAL_SCENARIO);
  if (descriptor.simulationMode === 'kinematic_pose') return structuredClone(UNITREE_G1_RIG_SCENARIO);
  if (descriptor.simulationMode === 'policy_sim') return structuredClone(MICRODUCK_SCENARIO);
  if (cache.has(descriptor.id)) return structuredClone(cache.get(descriptor.id));
  const response = await fetch(descriptor.url, { cache: 'force-cache' });
  if (!response.ok) throw new Error(`Pinned task ${descriptor.id} returned HTTP ${response.status}.`);
  const scenario = await response.json();
  if (scenario?.schema !== 'robobuddy.lab-scenario.v2') throw new Error(`${descriptor.id}: unexpected scenario schema.`);
  if (scenario.id !== descriptor.id) throw new Error(`${descriptor.id}: pinned task id mismatch (${scenario.id || 'missing'}).`);
  if (scenario.robotId !== descriptor.robotId) throw new Error(`${descriptor.id}: robot id mismatch (${scenario.robotId || 'missing'}).`);
  if (scenario.title !== descriptor.title) throw new Error(`${descriptor.id}: reviewed title mismatch; refusing silent task drift.`);
  const actions = scenario?.portablePython?.referenceActions;
  if (!Array.isArray(actions) || actions.length < 2) throw new Error(`${descriptor.id}: reviewed portablePython.referenceActions are missing.`);
  for (const [index, item] of actions.entries()) {
    if (!item || typeof item.action !== 'object' || Array.isArray(item.action)) throw new Error(`${descriptor.id}: reference action ${index + 1} is invalid.`);
    if (!Number.isFinite(Number(item.hold_seconds))) throw new Error(`${descriptor.id}: reference action ${index + 1} has no finite hold_seconds.`);
  }
  cache.set(descriptor.id, scenario);
  return structuredClone(scenario);
}

export function taskPatchProvenance(descriptor) {
  if (descriptor?.simulationMode === 'physical_mujoco') {
    return {
      repository: 'jivishov/RoboBuddy_IDE_v020',
      scenarioId: descriptor.id,
      physicalSceneId: descriptor.physicalSceneId,
      modelPackage: SO101_PHYSICAL_SCENARIO.modelPackage,
      simulationMode: 'physical_mujoco',
    };
  }
  if (descriptor?.simulationMode === 'kinematic_pose') {
    return {
      repository: 'jivishov/RoboBuddy_AI',
      revision: UNITREE_G1_VISUAL_REVISION,
      scenarioId: descriptor.id,
      source: descriptor.source,
      simulationMode: 'kinematic_pose',
    };
  }
  return descriptor ? {
    repository: TASK_PATCH_SOURCE,
    revision: TASK_PATCH_REVISION,
    scenarioId: descriptor.id,
    source: descriptor.url,
  } : null;
}

export function isPhysicalMujocoScenario(scenario) {
  return scenario?.simulationMode === 'physical_mujoco';
}

export function isKinematicRigScenario(scenario) {
  return scenario?.simulationMode === 'kinematic_pose';
}
