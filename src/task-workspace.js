export * from './task-workspace-base.js';
import { buildPatchedWorkspace as buildBaseWorkspace } from './task-workspace-base.js';
import { OPENARM_LAB_SCENE_VERSION, OPENARM_LAB_TASK_VERSION } from './physics/openarm-lab-builder.js';

function labBuilderWorkspace(scenario) {
  const main = [
    '# OpenArm image-assisted Lab Builder.',
    '# The physical workspace starts with OpenArm, its declared fixed mount/pedestal, and ground only.',
    '# A multimodal agent interprets the reference image externally and authors SceneSpec through WebMCP.',
    '# This starter program does not reconstruct a hidden reference task and does not auto-run a robot task.',
    'from robobuddy.sim import connect',
    'from robot_config import ROBOT_ID',
    '',
    'robot = await connect(ROBOT_ID)',
    'try:',
    '    observation = await robot.get_observation()',
    '    print("Lab Builder ready at simulation time", observation["simulationTimeSeconds"])',
    '    print("Authored bodies", [name for name in observation["bodies"] if name.startswith("lab_")])',
    '    print("Use the bounded Lab Builder WebMCP tools to stage/apply a SceneSpec, freeze a TaskSpec, plan, and then edit/run the returned program.")',
    'finally:',
    '    await robot.disconnect()',
    '',
  ].join('\n');
  const project = [
    '# Persistent project declarations are exported by the Lab Builder tool.',
    '# The page does not need the reference-image pixels when the external multimodal agent has already interpreted them.',
    '# A chat attachment id is not treated as a browser-readable file and is never fabricated into a content hash.',
    `SCENE_SCHEMA = ${JSON.stringify(OPENARM_LAB_SCENE_VERSION)}`,
    `TASK_SCHEMA = ${JSON.stringify(OPENARM_LAB_TASK_VERSION)}`,
    'SCENE_SPEC = None',
    'TASK_SPEC = None',
    'PROGRAM = None',
    '',
  ].join('\n');
  const config = [
    '# Browser simulation only. No serial, CAN or hardware connection is opened.',
    `ROBOT_ID = ${JSON.stringify(scenario.robotId)}`,
    'PHYSICAL_API_VERSION = "robobuddy.sim.v1"',
    'LENGTH_UNIT = "m"',
    'ANGLE_UNIT = "rad"',
    'TIME_UNIT = "s"',
    'OBSERVATION_MODE = "simulator_ground_truth"',
    'HARDWARE_VALIDATED = False',
    '',
  ].join('\n');
  const workcell = [
    '# Standalone Lab Builder boundary and capability statement.',
    `WORKCELL = ${JSON.stringify({
      scenario_id: scenario.id,
      physical_scene_id: scenario.physicalSceneId,
      workspace_mode: 'lab_builder',
      empty_start: ['ground', 'source-derived OpenArm V2 bimanual system', 'declared world-fixed mount/pedestal'],
      preloaded_lab_assets: [],
      supported: scenario.capabilities?.supported || [],
      unavailable: scenario.capabilities?.unavailable || [],
      hardware_validated: false,
    }, null, 2)}`.replace(/\btrue\b/g, 'True').replace(/\bfalse\b/g, 'False').replace(/\bnull\b/g, 'None'),
    '',
  ].join('\n');
  return { 'main.py': main, 'lab_project.py': project, 'robot_config.py': config, 'workcell.py': workcell };
}

export function buildPatchedWorkspace(profileId, scenario) {
  if (profileId === 'openarm' && scenario?.labBuilder === true) return labBuilderWorkspace(scenario);
  return buildBaseWorkspace(profileId, scenario);
}
