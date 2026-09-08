import { TASK_PATCH_REVISION, TASK_PATCH_SOURCE } from './task-catalog.js';

const isKinematicPoseScenario = (scenario) => scenario?.simulationMode === 'kinematic_pose';
const isPhysicalMujocoScenario = (scenario) => scenario?.simulationMode === 'physical_mujoco';

function py(value, depth = 0) {
  if (value === null || value === undefined) return 'None';
  if (typeof value === 'boolean') return value ? 'True' : 'False';
  if (typeof value === 'number') return Number.isFinite(value) ? String(value) : 'None';
  if (typeof value === 'string') return JSON.stringify(value);
  const indent = '    '.repeat(depth); const child = '    '.repeat(depth + 1);
  if (Array.isArray(value)) return value.length ? `[\n${value.map((item) => `${child}${py(item, depth + 1)},`).join('\n')}\n${indent}]` : '[]';
  const entries = Object.entries(value); if (!entries.length) return '{}';
  return `{\n${entries.map(([key, item]) => `${child}${JSON.stringify(key)}: ${py(item, depth + 1)},`).join('\n')}\n${indent}}`;
}

function trajectoryModule(scenario) {
  const header = isKinematicPoseScenario(scenario)
    ? ['# Unitree G1 kinematic-pose inspection trace.', `# Canonical mesh source: ${scenario.canonicalModel.repository}@${scenario.canonicalModel.revision}`, `# Scenario: ${scenario.id} — ${scenario.title}`, '# Every action is a bounded named joint-angle dictionary for the browser pose rig.', '# This is not a Unitree SDK, collision/contact plant, gait, balance, or hardware-control program.', ''].join('\n')
    : ['# Reviewed atomic physical-target action trace.', `# Source: ${TASK_PATCH_SOURCE}@${TASK_PATCH_REVISION}`, `# Scenario: ${scenario.id} — ${scenario.title}`, '# Every action below is an ordinary public robot.send_action() dictionary.', '# Edit the numerical joint/gripper/base targets here and immediately rerun the simulation.', '# RoboBuddy source generated this trace through its reviewed reference plant;', '# edits made here are revalidated by that same pinned source plant during simulation.', ''].join('\n');
  const rows = scenario.portablePython.referenceActions.map((record, index) => ({ index: index + 1, label: String(record.label || `action ${index + 1}`), hold_seconds: Number(record.hold_seconds), action: record.action }));
  return `${header}REFERENCE_ACTIONS = ${py(rows)}\n`;
}

function physicalWorkspace(scenario) {
  const dt = Number(scenario?.physicalApi?.timestepSeconds);
  if (!Number.isFinite(dt) || dt <= 0) throw new Error(`${scenario.id}: physicalApi.timestepSeconds is required`);
  const stages = scenario.portablePython.referenceActions.map((record, index) => ({ index: index + 1, label: String(record.label), duration_seconds: Number(record.hold_seconds), targets_rad: record.targetsRad || {} }));
  const staged = stages.map((stage, index) => {
    if (!Object.keys(stage.targets_rad).length) return { ...stage, max_steps: null };
    let budgetSeconds = stage.duration_seconds;
    for (let cursor = index + 1; cursor < stages.length && !Object.keys(stages[cursor].targets_rad).length; cursor += 1) budgetSeconds += stages[cursor].duration_seconds;
    return { ...stage, max_steps: Math.max(1, Math.ceil(budgetSeconds / dt) + 50) };
  });
  return {
    'main.py': `# ${scenario.title} — physical MuJoCo workspace.\n# robobuddy.sim.v1 uses SI units, radians, and explicit simulation time.\n# send_action() only latches bounded actuator targets; it is not target achievement.\nfrom robobuddy.sim import connect\nfrom robot_config import ROBOT_ID\nfrom trajectories import STAGES\n\nrobot = await connect(ROBOT_ID)\n\ntry:\n    for stage in STAGES:\n        targets = stage["targets_rad"]\n        if targets:\n            accepted = await robot.send_action(targets, max_steps=stage["max_steps"])\n            print(f'{stage["index"]:02d} {stage["label"]} accepted', accepted["status"])\n        observation = await robot.advance(stage["duration_seconds"])\n        print(f'{stage["index"]:02d} {stage["label"]} actual t={observation["simulationTimeSeconds"]:.3f}s contacts={observation["contactCount"]}')\n\n    final_observation = await robot.get_observation()\n    print("final actual bodies", final_observation["bodies"])\nfinally:\n    await robot.disconnect()\n`,
    'trajectories.py': `# Validated physical controller stages for ${scenario.id}.\n# Joint targets are radians. Durations are simulation seconds, never wall-clock sleeps.\n# No object is attached, parented, welded, snapped, teleported, or directly transformed.\nSTAGES = ${py(staged)}\n`,
    'robot_config.py': `# Browser-only physical simulation identity. No hardware transport is opened.\nROBOT_ID = ${JSON.stringify(scenario.robotId)}\nPHYSICAL_API_VERSION = ${JSON.stringify(scenario.physicalApi.version)}\nANGLE_UNIT = "rad"\nTIME_UNIT = "s"\nLENGTH_UNIT = "m"\nTIMESTEP_SECONDS = ${dt}\n`,
    'workcell.py': `# Declared physical-workspace metadata. Estimated benchmark parameters are\n# simulator evidence only and are not installed-hardware calibration.\nWORKCELL = ${py({ scenario_id: scenario.id, legacy_task_id: scenario.legacyTaskId || null, title: scenario.title, simulation_mode: scenario.simulationMode, physical_scene_id: scenario.physicalSceneId, model_package: scenario.modelPackage, model_id: scenario.modelId, evaluator: scenario.taskEvaluation, limitations: scenario.limitations })}\n`,
  };
}

function openArmConfig() { return `from lerobot.robots.openarm_follower import OpenArmFollowerConfigBase\nfrom lerobot.robots.bi_openarm_follower import BiOpenArmFollower, BiOpenArmFollowerConfig\n\nLEFT_CAN = "can0"\nRIGHT_CAN = "can1"\n\ndef create_robot():\n    config = BiOpenArmFollowerConfig(\n        left_arm_config=OpenArmFollowerConfigBase(port=LEFT_CAN, side="left", cameras={}),\n        right_arm_config=OpenArmFollowerConfigBase(port=RIGHT_CAN, side="right", cameras={}),\n        cameras={},\n    )\n    return BiOpenArmFollower(config)\n`; }
function so101Config() { return `from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig\n\nSERIAL_PORT = "/dev/ttyACM0"\n\ndef create_robot():\n    return SO101Follower(SO101FollowerConfig(port=SERIAL_PORT, cameras={}))\n`; }
function lekiwiConfig() { return `from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig\n\nROBOT_IP = "192.168.4.1"\n\ndef create_robot():\n    return LeKiwiClient(LeKiwiClientConfig(remote_ip=ROBOT_IP, cameras={}))\n`; }
function unitreeConfig() { return `# Browser-only adapter used by this kinematic visual workspace.\n# It is deliberately not a Unitree SDK or hardware controller.\nfrom robobuddy.simulation import UnitreeG1KinematicPoseAdapter\n\ndef create_robot():\n    return UnitreeG1KinematicPoseAdapter()\n`; }
function microduckConfig() { return `# Browser-only MicroDuck policy-simulation client.\n# connect() acquires a simulator lease; it never opens a socket or discovers hardware.\nfrom microduck import MicroDuck\n\ndef create_robot():\n    return MicroDuck()\n`; }

function microduckWorkspace(scenario) {
  return {
    'main.py': `# Live browser-only MicroDuck policy simulation.\n# Exact pinned ONNX policies drive original approximate browser dynamics; this is not hardware or RL-environment parity.\nfrom robot_config import create_robot\n\nrobot = create_robot()\nawait robot.connect()\ntry:\n    await robot.enable(True)\n    for _ in range(16):\n        await robot.move(0.30, 0.0, 0.0)\n        await robot.sleep(0.25)\n    state = await robot.get_state()\n    print("live modeled time", state["time"], "mode", state["mode"], "position", state["simulatedPose"]["position"])\n    await robot.stop()\nfinally:\n    await robot.disconnect()\n`,
    'trajectories.py': `# Callable examples for the catalog-backed robot methods. They are not run by main.py.\nasync def demonstrate_motion(robot):\n    await robot.head(neck_pitch=0.05, head_pitch=-0.1, head_yaw=0.2, head_roll=0.0)\n    await robot.look(0.35, 0.05, 0.18, neck_pitch=0.0)\n    await robot.pose(z=-0.01, roll=0.04, pitch=-0.04, active=True)\n    await robot.mouth(0.35)\n    await robot.do("kick_left")\n    await robot.init()\n    await robot.relax()\n\nasync def demonstrate_modes_and_audio(robot):\n    current = await robot.mode()\n    await robot.set_mode("roller" if current == "walking" else "walking")\n    await robot.sound("chirp")\n    await robot.sound("wheee", hold=True)\n    await robot.sound("wheee", hold=False)\n    await robot.theremin(True)\n    await robot.theremin(False)\n    await robot.chorale(True, piece="wistful", voices=2)\n    await robot.chorale(False)\n`,
    'robot_config.py': microduckConfig(),
    'workcell.py': `# Callable examples for browser-model presentation and workcell extensions.\n# Camera, ToF, contacts, geometry, rollers, mouth pivot, and dynamics are modeled approximations.\nWORKCELL = ${py({ scenario_id: scenario.id, simulation_mode: 'policy_sim', fidelity: 'exact pinned ONNX over original approximate browser dynamics; no RL-environment, locomotion, contact, or hardware parity' })}\n\nCAMERA_VIEWS = {"orbit": "Overview: world-frame robot + ball context", "chase": "Follow: stable robot-root third-person view", "head": "Head POV: modeled head_camera render, not hardware video"}\n\nasync def configure_modeled_workcell(robot):\n    await robot.set_color("lavender")\n    await robot.spawn_ball([0.28, 0.0, 0.035])\n    await robot.set_tof_stimulus(0.32)\n    await robot.set_camera("chase")\n    camera = (await robot.get_state())["virtualCamera"]\n    print("camera", camera["name"], camera["frame"], camera["purpose"])\n    await robot.reset()\n`,
  };
}

function mainFile(scenario) {
  const boundary = isKinematicPoseScenario(scenario)
    ? '# This is a browser-only Unitree G1 kinematic-pose workspace. The adapter\n# only displays bounded source-manifest joint angles; it is not a Unitree SDK,\n# contact simulation, gait/balance system, or hardware-control path.'
    : '# This is physical-target Python. The browser simulates the same public\n# send_action/get_observation sequence; it does not add grasp(), attach(),\n# teleport(), or Cartesian convenience methods to the learner program.';
  return `import time\n\nfrom robot_config import create_robot\nfrom trajectories import REFERENCE_ACTIONS\n\n# ${scenario.title}\n${boundary}\n\nrobot = create_robot()\nrobot.connect()\n\ntry:\n    for step in REFERENCE_ACTIONS:\n        sent = robot.send_action(step["action"])\n        time.sleep(step["hold_seconds"])\n        observation = robot.get_observation()\n        print(f'{step["index"]:02d} {step["label"]}', observation)\nfinally:\n    robot.disconnect()\n`;
}
function workcellFile(scenario) {
  const source = isKinematicPoseScenario(scenario) ? { repository: scenario.canonicalModel.repository, revision: scenario.canonicalModel.revision } : { repository: TASK_PATCH_SOURCE, revision: TASK_PATCH_REVISION };
  const summary = { scenario_id: scenario.id, title: scenario.title, robot_id: scenario.robotId, canonical_model: scenario.canonicalModel, frames: scenario.frames, source_repository: source.repository, source_revision: source.revision, simulation_mode: scenario.simulationMode || 'source_plant' };
  return `# Read-only reference geometry copied from the pinned reviewed mission.\n# Robot motion is NOT loaded from this file; motion remains visible in trajectories.py.\nWORKCELL = ${py(summary)}\n`;
}

export function buildPatchedWorkspace(profileId, scenario) {
  if (!scenario) throw new Error('A scenario is required to build the workspace.');
  if (isPhysicalMujocoScenario(scenario)) return physicalWorkspace(scenario);
  if (scenario.simulationMode === 'policy_sim') return microduckWorkspace(scenario);
  const configs = { openarm: openArmConfig, so101: so101Config, lekiwi: lekiwiConfig, unitree: unitreeConfig };
  const configFactory = configs[profileId]; if (!configFactory) throw new Error(`No workspace generator for ${profileId}.`);
  return { 'main.py': mainFile(scenario), 'trajectories.py': trajectoryModule(scenario), 'robot_config.py': configFactory(), 'workcell.py': workcellFile(scenario) };
}
