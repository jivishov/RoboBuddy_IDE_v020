import { ASIMOV_SCENES } from './asimov-scene.js';
import { ASIMOV_PACKAGES,ASIMOV_LIMITATIONS } from './asimov-model-package.js';
import { ASIMOV_SOURCE } from './asimov-generated.js';
const labels={mounted:'Asimov 1 — Mounted Joint Lab',freebase:'Asimov 1 — Free-base Dynamics',drop:'Asimov 1 — Passive Gravity Drop'};
export const ASIMOV_WORKSPACES=Object.freeze(Object.fromEntries(Object.entries(ASIMOV_SCENES).map(([variant,s])=>[s.id,Object.freeze({
  schema:'robobuddy.physical-workspace.v1',schemaVersion:1,simulationMode:'physical_mujoco',profileId:'asimov',id:s.id,title:labels[variant],
  brief:variant==='mounted'?'The pelvis is explicitly fixed to a visible mount. Explore real joint dynamics, bounded torques and measured tracking; this is not standing evidence.':
    variant==='drop'?'An explicitly elevated, passive free-base robot falls under gravity and collides with the floor. No controller, root correction or reset-as-recovery.':
    'A free-base Asimov under real gravity, source contacts and estimated joint-space PD. It may fall; no verified balance or walking policy is supplied.',
  robotId:s.robotId,workspaceRevision:s.revision,physicalSceneId:s.id,physicalSceneRevision:s.revision,modelPackage:s.modelPackage,modelId:ASIMOV_PACKAGES[variant].modelId,
  physicalApi:{version:'robobuddy.sim.v1',angleUnit:'rad',timeUnit:'s',lengthUnit:'m',torqueUnit:'N*m'},
  canonicalModel:{repository:'menloresearch/asimov-1',revision:ASIMOV_SOURCE.revision,sourceMjcfSha256:ASIMOV_SOURCE.sourceXmlSha256,authority:'Observed MuJoCo world-body transforms only'},
  controller:{physicsTimestepSeconds:.005,controlIntervalSeconds:.005,claim:'Repository-estimated PD, not hardware calibration'},
  capabilities:{jointTargets:'supported',gravity:'supported',contacts:'source-primitives',walking:'unsupported',grasping:'unsupported',balanceRecovery:'unsupported',neck:'fixed in source'},
  taskEvaluation:{type:'observation-only',syntheticSuccessEvents:false},limitations:ASIMOV_LIMITATIONS,
})])));
// Mounted lab is the useful beginner default; free-base and passive drop remain first-class scenes.
export const ASIMOV_TASKS=Object.freeze(['mounted','freebase','drop'].map(v=>ASIMOV_WORKSPACES[`asimov-${v}`]));
export function asimovWorkspaceFiles(scenario) {
  const passive=scenario.id==='asimov-drop';
  const main=[
    '# Simulation only: no serial, CAN, DDS or physical robot is connected.',
    '# All joint targets use radians. Requested targets are not measured positions.',
    'from robobuddy.sim import connect','from robot_config import ROBOT_ID','',
    'robot = await connect(ROBOT_ID)','try:',
    ...(passive?[]:['    await robot.set_joint_targets({"left_elbow_joint": 1.0, "right_elbow_joint": -1.0})']),
    '    for _ in range(10):','        await robot.wait_sim(0.1)','        state = await robot.get_state()',
    '        print("t", state["simulation_time_s"], "root", state["root"]["position_m"],',
    '              "elbow", state["joints"]["left_elbow_joint"]["position_rad"])',
    'finally:','    await robot.disconnect()','',
  ].join('\n');
  return {'main.py':main,'robot_config.py':`ROBOT_ID = ${JSON.stringify(scenario.robotId)}\n# 23 source hinges; neck fixed. Hardware model parameters are not calibrated.\n`,
    'trajectories.py':'# Edit main.py to command source-named joints in radians. No walk or grasp operation is supplied.\n',
    'workcell.py':`# Declared physical scene: ${scenario.physicalSceneId}\n# ${scenario.brief}\n`};
}
