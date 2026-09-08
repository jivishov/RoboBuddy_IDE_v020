import fs from 'node:fs';
import {
  OPENARM_PHYSICAL_TASKS,
  PATCH_TASKS,
  SO101_LEGACY_TASK_CLASSIFICATION,
  TASK_PATCH_REVISION,
  loadPatchedScenario,
  tasksForProfile,
} from '../src/task-catalog.js';
import { validateAction } from '../src/profiles.js';
import { buildPatchedWorkspace } from '../src/task-workspace.js';

if (TASK_PATCH_REVISION !== '75fe2669c0ab0b029986de424c69162071174df8') throw new Error('unexpected task patch revision');
const expected = {
  openarm: ['openarm-04-filtration-workcell'],
  so101: ['so101-v2-06-quantitative-transfer','so101-v2-08-burette-initial-reading','so101-v2-09-vacuum-filtration'],
  lekiwi: ['lekiwi-01-beaker-courier'],
};
for (const [profileId, ids] of Object.entries(expected)) {
  const configured = (PATCH_TASKS[profileId] || []).map((item) => item.id);
  if (JSON.stringify(configured) !== JSON.stringify(ids)) throw new Error(`${profileId} task catalog drift: ${configured}`);
  if (profileId === 'openarm') continue; // OpenArm descriptor remains pinned provenance; the normal catalog is now the physical V2 migration.
  for (const id of ids) {
    const scenario = await loadPatchedScenario(profileId, id);
    const actions = scenario.portablePython.referenceActions;
    if (!actions.length) throw new Error(`${id}: no reference actions`);
    for (const [index, record] of actions.entries()) {
      try { validateAction(profileId, record.action); }
      catch (error) { throw new Error(`${id} action ${index + 1} violates physical public API envelope: ${error.message}`); }
    }
    const workspace = buildPatchedWorkspace(profileId, scenario);
    for (const file of ['main.py','trajectories.py','robot_config.py','workcell.py']) if (!workspace[file]) throw new Error(`${id}: missing ${file}`);
    if (/\.(grasp|attach|teleport|move_to)\s*\(/.test(workspace['main.py'])) throw new Error(`${id}: fake physical robot method exposed`);
  }
}

const visibleOpenarm = tasksForProfile('openarm');
if (visibleOpenarm.length !== 1 || visibleOpenarm[0].id !== 'openarm-04-filtration-workcell' || visibleOpenarm[0].simulationMode !== 'physical_mujoco') throw new Error(`OpenArm physical catalog drift: ${JSON.stringify(visibleOpenarm)}`);
if (OPENARM_PHYSICAL_TASKS[0].physicalSceneId !== 'phase5a-openarm-v2-bimanual-stack') throw new Error('OpenArm physical scene id drifted');
const openarmScenario = await loadPatchedScenario('openarm', 'openarm-04-filtration-workcell');
if (openarmScenario.modelPackage !== 'openarm-v2-phase5a-a8c9796-v1') throw new Error('OpenArm physical model package drifted');
if (openarmScenario.robotId !== 'openarm_v2_bimanual') throw new Error('OpenArm physical robot identity drifted');
const openarmWorkspace = buildPatchedWorkspace('openarm', openarmScenario);
for (const token of ['from robobuddy.sim import connect', 'await connect(', 'await robot.send_action(', 'await robot.advance(', 'await robot.get_observation()']) if (!openarmWorkspace['main.py'].includes(token)) throw new Error(`OpenArm live physical starter missing ${token}`);
for (const forbidden of ['time.sleep(', 'lerobot', '.grasp(', '.attach(', '.teleport(', '.move_to(']) if (openarmWorkspace['main.py'].includes(forbidden)) throw new Error(`OpenArm physical starter exposes forbidden/legacy behavior ${forbidden}`);
if (!openarmWorkspace['trajectories.py'].includes('openarm_left_joint1') || !openarmWorkspace['trajectories.py'].includes('openarm_right_joint1')) throw new Error('OpenArm trajectories must expose both V2 arms as radian targets');
if (!openarmWorkspace['workcell.py'].includes('calibration')) throw new Error('OpenArm workcell must preserve calibration boundary');

const visibleSo101 = tasksForProfile('so101');
if (visibleSo101.length !== 1 || visibleSo101[0].id !== 'so101-physical-block-transfer' || visibleSo101[0].simulationMode !== 'physical_mujoco') throw new Error(`SO-101 physical catalog must expose only the validated block transfer: ${JSON.stringify(visibleSo101)}`);
for (const legacyId of expected.so101) {
  if (SO101_LEGACY_TASK_CLASSIFICATION[legacyId]?.classification !== 'B_DEFERRED') throw new Error(`${legacyId}: legacy physical classification must remain deliberately deferred`);
  if (visibleSo101.some((item) => item.id === legacyId)) throw new Error(`${legacyId}: deferred legacy task leaked into the physical SO-101 catalog`);
}
const physicalScenario = await loadPatchedScenario('so101', 'so101-physical-block-transfer');
if (physicalScenario.modelPackage !== 'so101-manipulation-menagerie-8161bba-v1') throw new Error('SO-101 physical task model package drifted');
const physicalWorkspace = buildPatchedWorkspace('so101', physicalScenario);
for (const forbidden of ['time.sleep(', 'lerobot', '.grasp(', '.attach(', '.teleport(', '.move_to(']) if (physicalWorkspace['main.py'].includes(forbidden)) throw new Error(`SO-101 physical starter exposes forbidden/legacy behavior ${forbidden}`);

const sourceSimulator = fs.readFileSync(new URL('../src/source-simulator.js', import.meta.url), 'utf8');
for (const token of ['ScenarioV2Engine.create', 'engine.plant.sendAction', 'engine.plant.tick()', 'engine.plant.fault']) if (!sourceSimulator.includes(token)) throw new Error(`source simulator missing ${token}`);
for (const token of ["kind: 'bimanual'", "side: 'bimanual'", 'connectionConfig']) if (!sourceSimulator.includes(token)) throw new Error(`OpenArm legacy source-plant provenance path missing ${token}`);
if (sourceSimulator.includes('left tool target would enter the modeled work surface')) throw new Error('stale standalone tool-point collision gate remains');

const host = fs.readFileSync(new URL('../src/simulator-host.js', import.meta.url), 'utf8');
for (const token of ['OpenArmPhysicalSimulator', "physical && profileId === 'openarm'", 'So101PhysicalSimulator']) if (!host.includes(token)) throw new Error(`simulator host missing physical routing token ${token}`);

const app = fs.readFileSync(new URL('../src/app-v2.js', import.meta.url), 'utf8');
for (const token of ['taskSelect', 'loadPatchedScenario', 'buildPatchedWorkspace', 'physicalRuntime.start', 'getPhysicalSession', 'isPhysicalWorkspace']) if (!app.includes(token)) throw new Error(`task-aware app missing ${token}`);

console.log('pinned legacy fixtures + SO-101/OpenArm physical catalogs/starters: OK');
