#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text()


def write(path, text):
    (ROOT / path).write_text(text)


def replace_once(text, old, new, label):
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


# 1. Physical presentation: authoritative base frame -> canonical URDF root, wheel rotation, audit.
path = 'src/physics/lekiwi-physical-simulator.js'
text = read(path)
text = replace_once(
    text,
    "import { MAX_WHEEL_RAD_S, publicActionToBodyCommand, WHEEL_ORDER } from './lekiwi-kinematics.js';\n",
    "import { MAX_WHEEL_RAD_S, publicActionToBodyCommand, WHEEL_ORDER } from './lekiwi-kinematics.js';\nimport { MODEL_FRAME } from './lekiwi-source-audit.js';\n",
    'lekiwi source-frame import',
)
text = replace_once(text, 'const PRESENTATION_GROUND_COLOR = 0x6b7377;', 'const PRESENTATION_GROUND_COLOR = 0x687378;', 'ground color')
text = replace_once(
    text,
    'const RAD_TO_DEG = 180 / Math.PI;\n',
    '''const RAD_TO_DEG = 180 / Math.PI;\n// The canonical mesh is baked in the pinned URDF root frame, while the physical MuJoCo base\n// body uses the audited LeRobot body frame at the wheel-centroid/axle origin.  The fixed\n// relationship is source-derived from MODEL_FRAME and is presentation-only.\nconst MODEL_ORIGIN_IN_URDF_M = Object.freeze(MODEL_FRAME.originInUrdfM.map(Number));\nconst URDF_ROOT_FROM_MODEL_ORIGIN_THREE_MM = Object.freeze([\n  -MODEL_ORIGIN_IN_URDF_M[1] * 1000,\n  -MODEL_ORIGIN_IN_URDF_M[2] * 1000,\n  -MODEL_ORIGIN_IN_URDF_M[0] * 1000,\n]);\nconst URDF_TO_MODEL_THREE_YAW_RAD = -Math.PI / 2;\n''',
    'presentation frame constants',
)
helper_marker = 'function finiteArmTargets(targetsRad) {'
helpers = '''function applyCanonicalBaseTransform(rig, observation) {\n  const base = observation?.bodies?.lekiwi_base;\n  if (!rig || !base?.positionM || !base?.quaternionWxyz) return false;\n  const baseThreeQuaternion = toThreeQuaternion(base.quaternionWxyz);\n  const offset = new THREE.Vector3(...URDF_ROOT_FROM_MODEL_ORIGIN_THREE_MM).applyQuaternion(baseThreeQuaternion);\n  const frameCorrection = new THREE.Quaternion().setFromAxisAngle(\n    new THREE.Vector3(0, 1, 0), URDF_TO_MODEL_THREE_YAW_RAD,\n  );\n  rig.root.position.copy(toThreePosition(base.positionM)).add(offset);\n  rig.root.quaternion.copy(baseThreeQuaternion).multiply(frameCorrection).normalize();\n  rig.root.updateMatrixWorld(true);\n  return true;\n}\n\nfunction applyCanonicalWheelState(rig, observation) {\n  if (!rig) return;\n  for (const wheelId of WHEEL_ORDER) {\n    const positionRad = Number(observation?.joints?.[wheelId]?.positionRad);\n    const group = rig.groups?.[wheelId];\n    const joint = group?.userData?.joint;\n    if (!Number.isFinite(positionRad) || !group || !joint) continue;\n    const axis = new THREE.Vector3().fromArray(joint.axis || [0, 1, 0]);\n    if (axis.lengthSq() < 1e-9) continue;\n    axis.normalize();\n    // The physical wheel joints use the audited LeRobot-positive axes, which are anti-parallel\n    // to the pinned URDF axes used by the canonical visual.  Reverse only the presentation angle.\n    const motion = new THREE.Quaternion().setFromAxisAngle(axis, -positionRad);\n    group.quaternion.copy(group.userData.baseQuaternion).multiply(motion).normalize();\n  }\n  rig.root.updateMatrixWorld(true);\n}\n\n'''
if helper_marker not in text:
    raise RuntimeError('finiteArmTargets marker missing')
text = text.replace(helper_marker, helpers + helper_marker, 1)
text = replace_once(
    text,
    "      payloadTransformSource: 'observed MuJoCo empty_beaker free-body pose',\n      rendererIntegratesBase: false,",
    "      payloadTransformSource: 'observed MuJoCo empty_beaker free-body pose',\n      wheelTransformSource: 'observed MuJoCo wheel joint positions mapped onto the anti-parallel pinned URDF visual axes',\n      visualRootFrameSource: MODEL_FRAME.urdfToModel,\n      visualRootOffsetThreeMm: [...URDF_ROOT_FROM_MODEL_ORIGIN_THREE_MM],\n      visualRootYawCorrectionRad: URDF_TO_MODEL_THREE_YAW_RAD,\n      rendererIntegratesBase: false,",
    'presentation audit fields',
)
alignment_method = '''  getPresentationAlignment() {\n    if (!this.canonicalRig || !this.lastObservation) return null;\n    if (this.presentationDirty) {\n      this.#applyObservation(this.lastObservation);\n      this.presentationDirty = false;\n    }\n    const pairs = [\n      ['shoulder_pan', 'arm_shoulder'],\n      ['shoulder_lift', 'arm_upper'],\n      ['elbow_flex', 'arm_lower'],\n      ['wrist_flex', 'arm_wrist'],\n      ['wrist_roll', 'arm_gripper_body'],\n    ];\n    const armPivotErrorsMm = {};\n    const visualPivotsMm = {};\n    const physicalPivotsMm = {};\n    for (const [visualId, bodyId] of pairs) {\n      const visual = this.canonicalRig.getWorldPosition(visualId);\n      const body = this.lastObservation.bodies?.[bodyId];\n      if (!visual || !body?.positionM) continue;\n      const physical = toThreePosition(body.positionM);\n      visualPivotsMm[visualId] = visual.toArray();\n      physicalPivotsMm[bodyId] = physical.toArray();\n      armPivotErrorsMm[visualId] = visual.distanceTo(physical);\n    }\n    const errors = Object.values(armPivotErrorsMm);\n    const beakerMesh = this.objectMeshes.get('empty_beaker');\n    const beakerBody = this.lastObservation.bodies?.empty_beaker;\n    const beakerPhysical = beakerBody?.positionM ? toThreePosition(beakerBody.positionM) : null;\n    const visualWrist = this.canonicalRig.getWorldPosition('wrist_roll');\n    return Object.freeze({\n      armPivotErrorsMm,\n      maxArmPivotErrorMm: errors.length ? Math.max(...errors) : null,\n      visualPivotsMm,\n      physicalPivotsMm,\n      beakerErrorMm: beakerMesh && beakerPhysical ? beakerMesh.position.distanceTo(beakerPhysical) : null,\n      visualWristToBeakerMm: visualWrist && beakerMesh ? visualWrist.distanceTo(beakerMesh.position) : null,\n      wheelPositionsRad: Object.fromEntries(WHEEL_ORDER.map((id) => [id, Number(this.lastObservation.joints?.[id]?.positionRad ?? 0)])),\n      wheelVisualQuaternions: Object.fromEntries(WHEEL_ORDER.map((id) => [id, this.canonicalRig.groups?.[id]?.quaternion?.toArray?.() || null])),\n      rootPositionMm: this.canonicalRig.root.position.toArray(),\n      rootQuaternion: this.canonicalRig.root.quaternion.toArray(),\n    });\n  }\n'''
text = replace_once(text, '  getTelemetry() {\n', alignment_method + '  getTelemetry() {\n', 'presentation alignment method')
old_apply = '''  #applyObservation(observation) {\n    if (this.canonicalRig) {\n      const base = observation.bodies?.lekiwi_base;\n      const pose = basePoseFromObservation(observation);\n      const basePose = pose\n        ? { x: pose.xM * 1000, z: -pose.yM * 1000, yaw: -pose.yawRad }\n        : null;\n      this.canonicalRig.applyPhysicalState(canonicalArmState(observation), basePose);\n      void base;\n    }\n    for (const [objectId, mesh] of this.objectMeshes) {\n      const body = observation.bodies?.[objectId];\n      if (!body?.positionM) continue;\n      mesh.position.copy(toThreePosition(body.positionM));\n      if (body.quaternionWxyz) mesh.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));\n    }\n  }\n'''
new_apply = '''  #applyObservation(observation) {\n    if (this.canonicalRig) {\n      // Arm joints, base frame and wheel spin are all presentation consumers of the same\n      // authoritative MuJoCo observation.  No presentation transform is fed back into physics.\n      this.canonicalRig.applyPhysicalState(canonicalArmState(observation));\n      applyCanonicalBaseTransform(this.canonicalRig, observation);\n      applyCanonicalWheelState(this.canonicalRig, observation);\n    }\n    for (const [objectId, mesh] of this.objectMeshes) {\n      const body = observation.bodies?.[objectId];\n      if (!body?.positionM) continue;\n      mesh.position.copy(toThreePosition(body.positionM));\n      if (body.quaternionWxyz) mesh.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));\n    }\n  }\n'''
text = replace_once(text, old_apply, new_apply, 'applyObservation')
write(path, text)

# 2. LeKiwi starter: keep simulation authoritative but yield visible physical frames during arm stages.
path = 'src/task-workspace.js'
text = read(path)
text = replace_once(
    text,
    "    'from robot_config import ROBOT_ID, CONTROLLER_PERIOD_S',",
    "    'from robot_config import ROBOT_ID, CONTROLLER_PERIOD_S, PHYSICS_TIMESTEP_S, PRESENTATION_PERIOD_S',",
    'starter config import',
)
text = replace_once(
    text,
    "    'from trajectories import STAGES, DRIVE_LIMITS, base_pose, chassis_command, waypoint_reached',\n    '',\n    'robot = await connect(ROBOT_ID)',",
    "    'from trajectories import STAGES, DRIVE_LIMITS, base_pose, chassis_command, waypoint_reached',\n    '',\n    'async def advance_visible(seconds):',\n    '    # Advance only through the authoritative simulation, but in bounded chunks so the browser',\n    '    # can render real intermediate MuJoCo observations instead of showing only stage endpoints.',\n    '    remaining_steps = max(0, int(round(seconds / PHYSICS_TIMESTEP_S)))',\n    '    chunk_steps = max(1, int(round(PRESENTATION_PERIOD_S / PHYSICS_TIMESTEP_S)))',\n    '    observation = await robot.get_observation()',\n    '    while remaining_steps > 0:',\n    '        steps = min(chunk_steps, remaining_steps)',\n    '        observation = await robot.advance(steps * PHYSICS_TIMESTEP_S)',\n    '        remaining_steps -= steps',\n    '    return observation',\n    '',\n    'robot = await connect(ROBOT_ID)',",
    'visible advance helper',
)
text = replace_once(
    text,
    "    '            observation = await robot.advance(stage[\"duration_seconds\"])',",
    "    '            observation = await advance_visible(stage[\"duration_seconds\"])',",
    'non-drive visible advance',
)
text = replace_once(
    text,
    "    'CONTROLLER_PERIOD_S = 0.02',\n    '',",
    "    'CONTROLLER_PERIOD_S = 0.02',\n    'PHYSICS_TIMESTEP_S = 0.002',\n    'PRESENTATION_PERIOD_S = 0.05',\n    '',",
    'presentation period config',
)
write(path, text)

# 3. Retire the legacy LeKiwi workspace from the selectable/resolvable task catalog.
path = 'src/task-catalog.js'
text = read(path)
pattern = re.compile(r"// The pinned legacy LeKiwi workspace stays selectable[\s\S]*?const LEKIWI_TASKS = Object\.freeze\(\[\.\.\.LEKIWI_PHYSICAL_TASKS, \.\.\.LEKIWI_LEGACY_TASKS\]\);\n")
replacement = "// The old source-plant LeKiwi scenario remains pinned only as provenance input in PATCH_TASKS/\n// LEGACY_TASK_SOURCE.  It is no longer a learner-selectable or resolvable LeKiwi workspace.\nconst LEKIWI_TASKS = LEKIWI_PHYSICAL_TASKS;\n"
text, count = pattern.subn(replacement, text, count=1)
if count != 1:
    raise RuntimeError(f'task catalog legacy block: {count}')
text = replace_once(
    text,
    "export async function loadPatchedScenario(profileId, taskId) {\n  const legacySo101 = profileId === 'so101' ? PATCH_TASKS.so101.find((item) => item.id === taskId) : null;\n  const legacyLekiwi = profileId === 'lekiwi' ? PATCH_TASKS.lekiwi.find((item) => item.id === taskId) : null;\n  const descriptor = legacySo101 || legacyLekiwi || taskDescriptor(profileId, taskId);",
    "export async function loadPatchedScenario(profileId, taskId) {\n  if (profileId === 'lekiwi' && taskId && taskId !== LEKIWI_PHYSICAL_TASKS[0].id) return null;\n  const legacySo101 = profileId === 'so101' ? PATCH_TASKS.so101.find((item) => item.id === taskId) : null;\n  const descriptor = legacySo101 || taskDescriptor(profileId, taskId);",
    'legacy LeKiwi resolver removal',
)
write(path, text)

# 4. Workspace/status copy must describe the selected physical robot, never copied SO-101/block text.
path = 'src/physics/ui-status.js'
text = read(path)
insert_after = '''const PHYSICAL_SIM_BADGE = Object.freeze({\n  openarm: 'MUJOCO V2 BIMANUAL DRY-STACK · SHARED FREE BODIES · NOT HARDWARE CALIBRATION',\n  lekiwi: 'MUJOCO HOLONOMIC BASE · PASSIVE OMNI ROLLERS · FREE PAYLOAD · NOT HARDWARE CALIBRATION',\n  so101: 'MUJOCO RIGID-BODY BENCHMARK · ACTUAL CONTACT/GRAVITY · NOT HARDWARE CALIBRATION',\n});\n'''
side_map = insert_after + '''\nconst PHYSICAL_SIDE_SUMMARY = Object.freeze({\n  openarm: 'OpenArm V2. Browser MuJoCo is the single physical authority for both arms and the free task objects; the canonical mesh is presentation-only and hardware validation remains pending.',\n  lekiwi: 'LeKiwi V1. Browser MuJoCo is the single physical authority for the free base, driven wheels, mounted arm and free beaker; the canonical mesh follows observed state only and hardware validation remains pending.',\n  so101: 'SO-101. Browser MuJoCo is the single physical authority for the arm and free benchmark object; the canonical mesh is presentation-only and hardware validation remains pending.',\n});\n'''
text = replace_once(text, insert_after, side_map, 'physical side summary map')
text = replace_once(
    text,
    "  const fidelityText = document.getElementById('fidelityText');\n",
    "  const fidelityText = document.getElementById('fidelityText');\n  const sideRobotSummary = document.getElementById('sideRobotSummary');\n",
    'side summary element',
)
text = replace_once(
    text,
    "  if (fidelityText && capability.backend === 'browser-mujoco' && profileId === 'openarm') {",
    "  if (sideRobotSummary && capability.backend === 'browser-mujoco' && PHYSICAL_SIDE_SUMMARY[profileId]) {\n    sideRobotSummary.textContent = PHYSICAL_SIDE_SUMMARY[profileId];\n  }\n  if (fidelityText && capability.backend === 'browser-mujoco' && profileId === 'openarm') {",
    'side summary application',
)
write(path, text)

# 5. Task/catalog tests: exactly one physical LeKiwi workspace; old id does not resolve.
path = 'tests/validate_task_patch.mjs'
text = read(path)
old_visible = '''const visibleLekiwi = tasksForProfile('lekiwi');\n// The physical workspace is the LeKiwi default; the pinned legacy source-plant workspace stays\n// selectable beside it and clearly labeled, so a physical load failure can never be mistaken for it.\nif (visibleLekiwi.length !== 2) throw new Error(`LeKiwi catalog must expose the physical and legacy workspaces: ${JSON.stringify(visibleLekiwi)}`);\nif (visibleLekiwi[0].id !== 'lekiwi-physical-beaker-courier' || visibleLekiwi[0].simulationMode !== 'physical_mujoco') throw new Error(`LeKiwi physical catalog drift: ${JSON.stringify(visibleLekiwi)}`);\nif (visibleLekiwi[1].id !== 'lekiwi-01-beaker-courier' || visibleLekiwi[1].simulationMode === 'physical_mujoco') throw new Error(`LeKiwi legacy workspace drift: ${JSON.stringify(visibleLekiwi)}`);\nif (!/legacy source plant/i.test(visibleLekiwi[1].title)) throw new Error(`LeKiwi legacy workspace must be labeled: ${visibleLekiwi[1].title}`);\n'''
new_visible = '''const visibleLekiwi = tasksForProfile('lekiwi');\nif (visibleLekiwi.length !== 1) throw new Error(`LeKiwi catalog must expose only the finalized physical workspace: ${JSON.stringify(visibleLekiwi)}`);\nif (visibleLekiwi[0].id !== 'lekiwi-physical-beaker-courier' || visibleLekiwi[0].simulationMode !== 'physical_mujoco') throw new Error(`LeKiwi physical catalog drift: ${JSON.stringify(visibleLekiwi)}`);\nif (visibleLekiwi.some((item) => item.id === 'lekiwi-01-beaker-courier')) throw new Error('legacy LeKiwi task leaked into the selectable catalog');\n'''
text = replace_once(text, old_visible, new_visible, 'visible LeKiwi task test')
old_legacy = '''const lekiwiLegacyScenario = await loadPatchedScenario('lekiwi', 'lekiwi-01-beaker-courier');\nif (!lekiwiLegacyScenario || lekiwiLegacyScenario.simulationMode === 'physical_mujoco') throw new Error('the pinned legacy LeKiwi workspace must still resolve to the source plant');\nif (!lekiwiLegacyScenario.portablePython?.referenceActions?.length) throw new Error('the legacy LeKiwi workspace must keep its pinned reference actions');\n'''
new_legacy = '''const lekiwiLegacyScenario = await loadPatchedScenario('lekiwi', 'lekiwi-01-beaker-courier');\nif (lekiwiLegacyScenario !== null) throw new Error('the retired legacy LeKiwi task id must no longer resolve as a workspace');\n'''
text = replace_once(text, old_legacy, new_legacy, 'legacy resolver test')
text = replace_once(
    text,
    "for (const token of ['from robobuddy.sim import connect', 'await connect(', 'await robot.send_action(', 'await robot.advance(', 'await robot.get_observation()', 'x.vel', 'waypoint_reached', 'base_pose']) {",
    "for (const token of ['from robobuddy.sim import connect', 'await connect(', 'await robot.send_action(', 'await robot.advance(', 'await robot.get_observation()', 'x.vel', 'waypoint_reached', 'base_pose', 'advance_visible', 'PRESENTATION_PERIOD_S']) {",
    'visible starter contract',
)
write(path, text)

# 6. Remove obsolete browser tests for the retired source-plant workspace while retaining Unitree coverage.
path = 'tests/browser-smoke.spec.mjs'
text = read(path)
start = text.index("test('legacy LeKiwi learner Python reaches the first action through the IDE Step Action path'")
end = text.index("test('Pause holds an active LeKiwi source-plant run and resumes it in place'", start)
text = text[:start] + text[end:]
start = text.index("test('Pause holds an active LeKiwi source-plant run and resumes it in place'")
end = text.index("test('LeKiwi source-plant and Unitree keep their main-thread compile/replay Run and Run-to-Cursor paths'", start)
text = text[:start] + text[end:]
start = text.index("test('LeKiwi source-plant and Unitree keep their main-thread compile/replay Run and Run-to-Cursor paths'")
end = text.index("test('diagnostics panel owns a full-width grid row and activity rail buttons are functional'", start)
unitree_test = '''test('Unitree keeps its main-thread compile/replay Run and Run-to-Cursor paths', async ({ page }) => {\n  test.setTimeout(240_000);\n  await page.goto('/?ci=cycle04-preservation', { waitUntil: 'domcontentloaded' });\n  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });\n  const setFirstActionCursor = () => page.evaluate(() => {\n    const app = window.__robobuddyCi.app;\n    const lines = app.files['main.py'].split('\\n');\n    const index = lines.findIndex((line) => line.includes('robot.send_action('));\n    app.openFile('main.py');\n    app.editor.cm.setCursor({ line: Math.max(0, index), ch: 0 });\n    return index + 1;\n  });\n\n  await page.selectOption('#robotSelect', 'unitree');\n  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 45_000 });\n  await page.click('#runBtn');\n  await expect(page.locator('#statusMessage')).toHaveText('Run complete', { timeout: SOURCE_REPLAY_TIMEOUT });\n  const unitreeLine = await setFirstActionCursor();\n  await page.click('#cursorBtn');\n  await expect(page.locator('#statusMessage')).toContainText(`main.py:${unitreeLine}`, { timeout: SOURCE_REPLAY_TIMEOUT });\n  expect(await page.evaluate(() => ({ policyWorker: window.__robobuddyCi.app.microduckRuntime.isActive(), prepared: window.__robobuddyCi.app.prepared?.events?.length > 0 }))).toEqual({ policyWorker: false, prepared: true });\n});\n\n'''
text = text[:start] + unitree_test + text[end:]
write(path, text)

# 7. Dedicated browser presentation test for the exact user-visible regressions.
presentation_test = r'''import { expect, test } from '@playwright/test';

test('LeKiwi canonical presentation follows the authoritative physical base, wheels, arm and beaker', async ({ page }) => {
  test.setTimeout(240_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/?ci=lekiwi-presentation', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('lekiwi');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 90_000 });

  const options = await page.locator('#taskSelect option').evaluateAll((nodes) => nodes.map((node) => ({ value: node.value, text: node.textContent })));
  expect(options).toEqual([{ value: 'lekiwi-physical-beaker-courier', text: 'Physical Beaker Courier' }]);
  await expect(page.locator('#sideRobotSummary')).toContainText('free base, driven wheels, mounted arm and free beaker');

  const initial = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    await sim.advanceTime(0.2);
    sim.renderFrame();
    return { alignment: sim.getPresentationAlignment(), audit: sim.getPresentationAudit(), mainPy: window.__robobuddyCi.app.files['main.py'], configPy: window.__robobuddyCi.app.files['robot_config.py'] };
  });
  expect(initial.audit.wheelTransformSource).toContain('observed MuJoCo wheel joint positions');
  expect(initial.audit.visualRootFrameSource).toContain('model_x = urdf_y');
  expect(initial.alignment.maxArmPivotErrorMm).not.toBeNull();
  expect(initial.alignment.maxArmPivotErrorMm).toBeLessThan(12);
  expect(initial.alignment.beakerErrorMm).toBeLessThan(1e-6);
  expect(initial.mainPy).toContain('advance_visible');
  expect(initial.configPy).toContain('PRESENTATION_PERIOD_S = 0.05');

  const driven = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const before = sim.getPresentationAlignment();
    await sim.applyChassisVelocity({ 'x.vel': 0.15, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 4000 });
    await sim.advanceTime(0.25);
    sim.renderFrame();
    const after = sim.getPresentationAlignment();
    await sim.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 2000 });
    return { before, after };
  });
  for (const wheel of ['base_left_wheel', 'base_right_wheel']) {
    expect(Math.abs(driven.after.wheelPositionsRad[wheel] - driven.before.wheelPositionsRad[wheel])).toBeGreaterThan(0.2);
    expect(driven.after.wheelVisualQuaternions[wheel]).not.toEqual(driven.before.wheelVisualQuaternions[wheel]);
  }

  const arm = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const { LEKIWI_ARM_POSES, LEKIWI_COURIER_CONTROLLER } = await import('/src/physics/lekiwi-scene.js');
    await sim.reset();
    await sim.advanceTime(0.2);
    sim.renderFrame();
    const before = sim.getPresentationAlignment();
    await sim.applyArmTargets({ ...LEKIWI_ARM_POSES.pick_grip, arm_gripper: LEKIWI_COURIER_CONTROLLER.gripperOpenRad }, { maxSteps: 8000 });
    const samples = [];
    for (let i = 0; i < 20; i += 1) {
      await sim.advanceTime(0.05);
      sim.renderFrame();
      samples.push(sim.getPresentationAlignment());
    }
    return { before, after: samples.at(-1), maxError: Math.max(...samples.map((sample) => sample.maxArmPivotErrorMm || 0)) };
  });
  expect(arm.maxError).toBeLessThan(12);
  expect(arm.after.beakerErrorMm).toBeLessThan(1e-6);
  expect(arm.after.visualWristToBeakerMm).toBeLessThan(180);
  const wristBefore = arm.before.visualPivotsMm.wrist_roll;
  const wristAfter = arm.after.visualPivotsMm.wrist_roll;
  expect(Math.hypot(...wristAfter.map((value, index) => value - wristBefore[index]))).toBeGreaterThan(40);

  expect(pageErrors).toEqual([]);
});
'''
write('tests/lekiwi-presentation-browser.spec.mjs', presentation_test)

# 8. P7 evidence: tighter wheel/traction sensitivity and presentation browser gate.
path = '.github/workflows/lekiwi-phase5b-validate.yml'
text = read(path)
text = replace_once(
    text,
    '          node --check tests/lekiwi-conformance-browser.spec.mjs\n',
    '          node --check tests/lekiwi-conformance-browser.spec.mjs\n          node --check tests/lekiwi-presentation-browser.spec.mjs\n',
    'presentation syntax gate',
)
text = replace_once(
    text,
    '          python native/lekiwi_reference.py --trial courier --timestep 0.001 > /tmp/lekiwi-courier-tight.json\n',
    '          python native/lekiwi_reference.py --trial courier --timestep 0.001 > /tmp/lekiwi-courier-tight.json\n          python native/lekiwi_reference.py --trial wheel-reference --timestep 0.001 > /tmp/lekiwi-wheel-reference-tight.json\n          python native/lekiwi_reference.py --trial traction-reduced --timestep 0.001 > /tmp/lekiwi-traction-reduced-tight.json\n',
    'tighter wheel evidence generation',
)
text = replace_once(
    text,
    "          tight_courier = load('courier-tight')\n",
    "          tight_courier = load('courier-tight')\n          tight_wheel = load('wheel-reference-tight')\n          tight_traction = load('traction-reduced-tight')\n",
    'load tighter wheel evidence',
)
text = replace_once(
    text,
    "          print('LeKiwi timestep sensitivity: OK', {'finalBeakerDeltaM': round(delta, 6)})\n",
    "          tw = tight_wheel['metrics']\n          assert abs(tw['grounded']['driveDirectionM'] - w['grounded']['driveDirectionM']) < 0.02, (w['grounded'], tw['grounded'])\n          assert abs(tw['anisotropyRatio'] - w['anisotropyRatio']) < 0.35, (w['anisotropyRatio'], tw['anisotropyRatio'])\n          ttr = tight_traction['metrics']\n          assert abs(ttr['travelRatio_0.5s'] - tr['travelRatio_0.5s']) < 0.08, (tr['travelRatio_0.5s'], ttr['travelRatio_0.5s'])\n          assert ttr['reduced_0.5s']['slipRatio'] > 0.5, ttr['reduced_0.5s']\n          print('LeKiwi timestep sensitivity: OK', {'finalBeakerDeltaM': round(delta, 6), 'wheelAnisotropy1ms': round(tw['anisotropyRatio'], 4)})\n",
    'tighter wheel assertions',
)
text = replace_once(
    text,
    "      - name: LeKiwi IDE physical end-to-end gate\n        run: npx playwright test tests/lekiwi-phase5b-browser.spec.mjs --reporter=line\n",
    "      - name: LeKiwi IDE physical end-to-end gate\n        run: npx playwright test tests/lekiwi-phase5b-browser.spec.mjs --reporter=line\n\n      - name: LeKiwi physical presentation alignment and wheel-animation gate\n        run: npx playwright test tests/lekiwi-presentation-browser.spec.mjs --reporter=line\n",
    'presentation browser workflow gate',
)
write(path, text)

# 9. Provenance language: simulator evidence is not hardware-invariant; document visual correction.
path = 'docs/physics/lekiwi-provenance.md'
text = read(path)
old_mass = '''The package preserves the URDF values unscaled rather than substituting an invented mass, and\nstates the consequence: **absolute mass, traction margin, motor loading, payload rating, and\nacceleration limits are calibration-required.** The P5B gates are relative and causal\n(motion arises from contact; a lifted base does not move; reduced traction degrades propulsion;\na payload is carried by contact), and none of them depends on the absolute mass being right.\n'''
new_mass = '''The package preserves the URDF values unscaled rather than substituting an invented mass, and\nstates the consequence: **absolute mass, traction margin, motor loading, payload rating, and\nacceleration limits are calibration-required.** The P5B gates establish causal behaviour of this\ndeclared simulation model (motion arises from contact; a lifted base does not move; reduced\ntraction degrades propulsion; a payload is carried by contact). They do **not** establish\nhardware-accurate acceleration, traction margin, motor loading, stopping distance, or payload\nperformance, because the absolute mass/inertia and several contact parameters remain uncalibrated.\n'''
text = replace_once(text, old_mass, new_mass, 'mass caveat wording')
old_cadence = '''A 10 ms cadence resolves the narrowest of these ≈25×. The full courier produces ≈3 270\nauthoritative samples per browser run.\n'''
new_cadence = '''A 10 ms cadence provides 25 samples across the 0.25 s evaluator settle dwell and two samples per\n20 ms controller period; the measured support/closed-grip overlap is much longer. The full courier\nproduces ≈3 270 authoritative samples per browser run.\n'''
text = replace_once(text, old_cadence, new_cadence, 'cadence wording')
frame_anchor = '''Forward is `+Y` in the URDF frame. That reading is supported three independent ways: the wheel the\nsource names `base_back_wheel` sits at `-Y`; the arm mounts at `+Y`; and with this convention the\nthree source wheel axes reproduce the pinned LeRobot Kiwi mapping angles exactly.\n'''
frame_extra = frame_anchor + '''\nThe canonical display mesh is baked in that pinned URDF root frame, whereas the MuJoCo free body\nuses the wheel-centroid LeRobot frame above. The physical renderer therefore applies the audited\nfixed URDF→model yaw and root-origin offset to every observed base pose before drawing the mesh.\nDisplayed wheel spin comes from the observed MuJoCo wheel `qpos`; because the physical joints use\nthe LeRobot-positive axes that are anti-parallel to the source URDF wheel axes, only the visual\nangle sign is reversed. These transforms are presentation-only and never feed state back to MuJoCo.\n'''
text = replace_once(text, frame_anchor, frame_extra, 'visual frame provenance')
legacy_anchor = '## 10. Task geometry provenance and repairs\n'
text = replace_once(
    text,
    legacy_anchor,
    legacy_anchor + '\nThe former `lekiwi-01-beaker-courier` source-plant workspace is no longer learner-selectable after P5B finalization. Its pinned scenario remains a provenance/reference input only; the selectable LeKiwi catalog contains only `lekiwi-physical-beaker-courier`.\n\n',
    'legacy task retirement provenance',
)
write(path, text)

print('LeKiwi finalization patch prepared successfully.')
