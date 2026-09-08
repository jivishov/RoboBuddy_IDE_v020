import fs from 'node:fs';
import {
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
    for (const firstKey of Object.keys(actions[0].action)) {
      if (!workspace['trajectories.py'].includes(firstKey)) throw new Error(`${id}: physical action field ${firstKey} is not visible in trajectories.py`);
    }
    if (/\.(grasp|attach|teleport|move_to)\s*\(/.test(workspace['main.py'])) throw new Error(`${id}: fake physical robot method exposed`);
  }
}

const visibleSo101 = tasksForProfile('so101');
if (visibleSo101.length !== 1 || visibleSo101[0].id !== 'so101-physical-block-transfer' || visibleSo101[0].simulationMode !== 'physical_mujoco') {
  throw new Error(`SO-101 physical catalog must expose only the validated block transfer: ${JSON.stringify(visibleSo101)}`);
}
for (const legacyId of expected.so101) {
  if (SO101_LEGACY_TASK_CLASSIFICATION[legacyId]?.classification !== 'B_DEFERRED') {
    throw new Error(`${legacyId}: legacy physical classification must remain deliberately deferred`);
  }
  if (visibleSo101.some((item) => item.id === legacyId)) throw new Error(`${legacyId}: deferred legacy task leaked into the physical SO-101 catalog`);
}
const physicalScenario = await loadPatchedScenario('so101', 'so101-physical-block-transfer');
if (physicalScenario.simulationMode !== 'physical_mujoco') throw new Error('SO-101 physical scenario is not physical_mujoco');
if (physicalScenario.modelPackage !== 'so101-manipulation-menagerie-8161bba-v1') throw new Error('SO-101 physical task model package drifted');
const physicalWorkspace = buildPatchedWorkspace('so101', physicalScenario);
for (const file of ['main.py','trajectories.py','robot_config.py','workcell.py']) if (!physicalWorkspace[file]) throw new Error(`SO-101 physical workspace missing ${file}`);
for (const token of ['from robobuddy.sim import connect', 'await connect(', 'await robot.send_action(', 'await robot.advance(', 'await robot.get_observation()']) {
  if (!physicalWorkspace['main.py'].includes(token)) throw new Error(`SO-101 live physical starter missing ${token}`);
}
for (const forbidden of ['time.sleep(', 'lerobot', '.grasp(', '.attach(', '.teleport(', '.move_to(']) {
  if (physicalWorkspace['main.py'].includes(forbidden)) throw new Error(`SO-101 physical starter exposes forbidden/legacy behavior ${forbidden}`);
}
if (!physicalWorkspace['trajectories.py'].includes('targets_rad')) throw new Error('SO-101 trajectories must expose radian physical targets');
if (!physicalWorkspace['workcell.py'].includes('synthetic') && !physicalWorkspace['workcell.py'].includes('Synthetic')) throw new Error('SO-101 workcell must label the benchmark as synthetic');

const sourceSimulator = fs.readFileSync(new URL('../src/source-simulator.js', import.meta.url), 'utf8');
for (const token of ['ScenarioV2Engine.create', 'engine.plant.sendAction', 'engine.plant.tick()', 'engine.plant.fault']) {
  if (!sourceSimulator.includes(token)) throw new Error(`source simulator missing ${token}`);
}
for (const token of ["kind: 'bimanual'", "side: 'bimanual'", 'connectionConfig']) {
  if (!sourceSimulator.includes(token)) throw new Error(`OpenArm source-plant connection missing ${token}`);
}
if (sourceSimulator.includes('left tool target would enter the modeled work surface')) throw new Error('stale standalone tool-point collision gate remains');

const app = fs.readFileSync(new URL('../src/app-v2.js', import.meta.url), 'utf8');
for (const token of ['taskSelect', 'loadPatchedScenario', 'buildPatchedWorkspace', 'physicalRuntime.start', 'getPhysicalSession', 'isPhysicalWorkspace']) if (!app.includes(token)) throw new Error(`task-aware app missing ${token}`);

for (const token of [
  "floor.name = 'presentation-ground-depth-biased'",
  'PRESENTATION_GROUND_COLOR = 0x687378',
  'color: PRESENTATION_GROUND_COLOR',
  'polygonOffset: true',
  'polygonOffsetFactor: 1',
  'polygonOffsetUnits: 4',
  'presentation-only-high-contrast-scene',
  "accessibilityRole: 'floor-contact-perimeter'",
  'collisionGeometry: false',
  'kinematics: false',
]) {
  if (!sourceSimulator.includes(token)) throw new Error(`presentation ground is missing z-fighting guard ${token}`);
}
if (sourceSimulator.includes('color: 0x17212b')) throw new Error('legacy dark presentation ground color remains');
for (const token of [
  'this.scene.background = new THREE.Color(0xb9c1c4)',
  'new THREE.HemisphereLight(0xffffff, 0x334155, 1.35)',
  'new THREE.DirectionalLight(0xffffff, 2.15)',
  'new THREE.DirectionalLight(0x93c5fd, 0.45)',
  'new THREE.GridHelper(1800, 36, 0x334155, 0x1f2937)',
]) {
  if (!sourceSimulator.includes(token)) throw new Error(`non-ground scene configuration drifted: ${token}`);
}

console.log('pinned legacy fixtures + SO-101 physical catalog/starter: OK');
