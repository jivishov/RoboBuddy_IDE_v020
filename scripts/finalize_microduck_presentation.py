from pathlib import Path


def replace(text, old, new, count=1):
    actual = text.count(old)
    if actual != count:
        raise RuntimeError(f'Expected {count} patch sites, got {actual}: {old[:100]}')
    return text.replace(old, new)

p = Path('src/microduck/rig-adapter.js')
s = p.read_text()
method = '''  /**
   * Render a single authoritative body-pose snapshot. Do not reconstruct joint FK here:
   * after mj_step the solved xpos/xquat/contact fields and integrated qpos belong to
   * different pipeline stages. Mixing them shifted moving feet relative to contacts.
   * All transforms below are presentation-only; the source meshes remain unmodified.
   */
  applyPhysicalBodyPoses(bodyPoses) {
    // Validate the complete snapshot BEFORE mutating any rendered node. Missing bodies
    // must fail visibly, never fall back to stale transforms or separately sampled joints.
    const poses = new Map(this.data.bodies.map(({ name }) => {
      const pose = bodyPoses?.[name];
      const p = pose?.positionM;
      const q = pose?.quaternionWxyz;
      if (!Array.isArray(p) || p.length !== 3 || !p.every(Number.isFinite)
        || !Array.isArray(q) || q.length !== 4 || !q.every(Number.isFinite)
        || Math.hypot(...q) < 1e-8) {
        throw new Error(`Missing or invalid authoritative MicroDuck body pose: ${name}`);
      }
      return [name, { position: new THREE.Vector3().fromArray(p), quaternion: quaternionFromWxyz(q) }];
    }));
    const local = this.data.bodies.map(({ name, parent }) => {
      const pose = poses.get(name);
      if (parent === null) return { name, position: new THREE.Vector3(), quaternion: new THREE.Quaternion() };
      const parentPose = poses.get(parent);
      if (!parentPose) throw new Error(`Missing authoritative MicroDuck parent pose: ${parent}`);
      const inverse = parentPose.quaternion.clone().invert();
      return { name,
        position: pose.position.clone().sub(parentPose.position).applyQuaternion(inverse),
        quaternion: inverse.multiply(pose.quaternion),
      };
    });
    const trunk = bodyPoses.trunk_base;
    this.applyPhysicalRootPose(trunk.positionM, trunk.quaternionWxyz);
    for (const { name, position, quaternion } of local) {
      const body = this.bodies.get(name);
      body.position.copy(position);
      body.quaternion.copy(quaternion);
    }
    this.root.updateWorldMatrix(true, true);
  }

'''
s = replace(s, '  getBounds(target = new THREE.Box3())', method + '  getBounds(target = new THREE.Box3())')
p.write_text(s)
p = Path('src/physics/microduck-physical-simulator.js')
s = p.read_text()
a = s.index('      const state = {};', s.index('  #applyPresentation(')); b = s.index('\n    }', a)
s = s[:a] + '      this.rig.applyPhysicalBodyPoses(observation.bodies);' + s[b:]
s = replace(s, "      jointPresentationSource: 'observed MuJoCo joint positions',", "      jointPresentationSource: 'same authoritative MuJoCo body-pose snapshot as trunk and contacts; no joint FK reconstruction',\n      bodyPoseSnapshotSource: 'MuJoCo xpos/xquat solved kinematics',")
s = replace(s, '''      this.robotRoot.add(this.rig.root);
    }
    this.rig.setVariant''', '''    }
    this.robotRoot.add(this.rig.root);
    this.rig.setVariant''')
p.write_text(s)
p = Path('src/physics/microduck-model-package.js')
s = p.read_text()
a=s.index('const ROBOT_BODIES = Object.freeze([');b=s.index('\n]);',a)+len('\n]);')
s=s[:a]+'''// Publish every source body, not just task-evaluation landmarks. The renderer consumes
// one coherent MuJoCo xpos/xquat snapshot instead of recomputing FK from separately integrated joint state.
const ROBOT_BODIES = Object.freeze([
  { id: MICRODUCK_TRUNK_BODY, freeJointId: 'trunk_base_freejoint' },
  ...['yaw2roll', 'hip_l', 'left_upper_leg', 'leg', 'ankle_left',
    'neck', 'neck_pitch', 'yaw_roll_motion', 'bottom_head_shell',
    'bearing_roll', 'hip_l_2', 'right_upper_leg', 'leg_2', 'ankle_right'].map((id) => ({ id })),
]);'''+s[b:]
p.write_text(s)
# The catalog test changes reflect explicit user retirement, not a relaxed physics gate.
p=Path('tests/physics/microduck-phase5c-core.mjs');s=p.read_text()
a=s.index("check('both MicroDuck workspaces");b=s.index("check('the physical MicroDuck workspace has",a)
s=s[:a]+'''check('MicroDuck exposes only physical task-specific workspaces and rejects the retired demonstrator', async () => {
  const { tasksForProfile, defaultTaskId, loadPatchedScenario } = await import('../../src/task-catalog.js');
  const { physicsCapabilityFor } = await import('../../src/physics/capabilities.js');
  const tasks = tasksForProfile('microduck');
  assert(tasks.length === 3, `MicroDuck must expose exactly three physical tasks, saw ${tasks.length}`);
  assert(tasks.map((item) => item.id).join(',') === 'microduck-physical-locomotion,microduck-physical-groundcontact,microduck-physical-kick', 'MicroDuck physical task order changed');
  assert(defaultTaskId('microduck') === 'microduck-physical-locomotion', 'Physical locomotion is not the default');
  for (const task of tasks) {
    assert(task.simulationMode === 'physical_mujoco', `${task.id} is not physical`);
    const scene = await loadPatchedScenario('microduck', task.id);
    assert(scene.simulationMode === 'physical_mujoco' && scene.id === task.id, 'Task must resolve to its own physical plant');
  }
  assert(await loadPatchedScenario('microduck', 'microduck-policy-demonstrator') === null, 'Retired demonstrator was silently remapped');
  assert(await loadPatchedScenario('microduck', 'not-a-microduck-task') === null, 'Unknown task was silently remapped');
  assert(physicsCapabilityFor('microduck', { physical: true }).backend === 'browser-mujoco', 'Physical backend evidence changed');
});

'''+s[b:];p.write_text(s)
p=Path('tests/physics/microduck-presentation-core.mjs')
p.write_text('''import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { MICRODUCK_MODEL_PACKAGES } from '../../src/physics/microduck-model-package.js';
const rig = JSON.parse(readFileSync(new URL('../../assets/microduck/generated/procedural-rig.json', import.meta.url)));
const names = rig.bodies.map(({ name }) => name);
assert.equal(names.length, 15);
for (const model of MICRODUCK_MODEL_PACKAGES) {
  assert.deepEqual(model.bodies.filter(({ id }) => id !== 'microduck_ball').map(({ id }) => id), names);
  const xml = readFileSync(new URL('../../' + model.asset, import.meta.url), 'utf8');
  assert.deepEqual([...xml.matchAll(/<body name="([^"]+)"/g)].map((m) => m[1]), model.bodies.map(({ id }) => id));
}
const simulator = readFileSync(new URL('../../src/physics/microduck-physical-simulator.js', import.meta.url), 'utf8');
const method = simulator.slice(simulator.indexOf('  #applyPresentation('), simulator.indexOf('  #disposePresentation('));
assert.match(method, /applyPhysicalBodyPoses\\(observation\\.bodies\\)/);
assert.doesNotMatch(method, /rig\\.applyState|observation\\.joints/);
assert.match(simulator, /includeConfiguredRollers: false/);
console.log('MicroDuck presentation contracts: all 15 source bodies in each plant; no mixed joint FK; no cosmetic rollers');
''')
p=Path('tests/microduck-presentation-browser.spec.mjs');s=p.read_text()
s=replace(s, '  expect(value.officialParts).toBe(58);', '  expect(value.bodyErrors).toHaveLength(15);\n  expect(value.officialParts).toBe(58);')
s=replace(s, '    rig.applyPhysicalRootPose(sim.lastObservation.bodies.trunk_base.positionM, sim.lastObservation.bodies.trunk_base.quaternionWxyz);', '    rig.applyPhysicalBodyPoses(sim.lastObservation.bodies);')
s += '''
test('physical MicroDuck rejects incomplete pose snapshots without partially moving its visual', async ({ page }) => {
  await openMicroDuck(page);
  const result = await page.evaluate(() => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const { rig } = sim;
    const transforms = () => rig.bodyList.map((body) => [body.position.toArray(), body.quaternion.toArray()]);
    const original = JSON.stringify(transforms());
    const physicalState = JSON.stringify(sim.lastObservation);
    const records = [];
    for (const kind of ['missing-body', 'non-finite', 'zero-quaternion']) {
      const snapshot = structuredClone(sim.lastObservation.bodies);
      snapshot.trunk_base.positionM[2] += 1;
      if (kind === 'missing-body') delete snapshot.ankle_right;
      if (kind === 'non-finite') snapshot.ankle_right.positionM[0] = NaN;
      if (kind === 'zero-quaternion') snapshot.ankle_right.quaternionWxyz = [0, 0, 0, 0];
      let rejected = false;
      try { rig.applyPhysicalBodyPoses(snapshot); } catch { rejected = true; }
      records.push({ kind, rejected, unchanged: original === JSON.stringify(transforms()) });
    }
    return { records, physicalUnchanged: physicalState === JSON.stringify(sim.lastObservation) };
  });
  expect(result.physicalUnchanged).toBe(true);
  for (const row of result.records) expect(row).toMatchObject({ rejected: true, unchanged: true });
});
'''
p.write_text(s)
# Preserve ONNX parity independently of the retired legacy UI journeys.
old=Path('tests/microduck-browser-smoke.spec.mjs').read_text()
a=old.index("test('vendored ORT matches walking and roller CPU fixtures'");b=old.index("test('MicroDuck control deck",a)
Path('tests/microduck-policy-parity.spec.mjs').write_text("import { expect, test } from '@playwright/test';\n\n"+old[a:b])
for name in ['microduck-browser-smoke.spec.mjs', 'microduck-policy-trace.spec.mjs', 'microduck-control-deck-browser-smoke.spec.mjs']:
    Path('tests', name).unlink()
p=Path('playwright.config.mjs');s=p.read_text().replace('microduck-policy-trace|','microduck-policy-parity|');p.write_text(s)
p=Path('tests/webmcp.spec.mjs');s=p.read_text();a=s.index("test('ready MicroDuck adds");b=s.index("test('loading and failed MicroDuck",a)
s=s[:a]+'''test('ready physical MicroDuck registers only its bounded physical tool and removes it on profile changes', async ({ page }) => {
  test.setTimeout(180_000);
  await openReadyApp(page);
  await page.locator('[data-agent-access="assist"]').click();
  await expect.poll(() => activeTools(page)).toEqual(OPENARM_TOOLS);
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await expect.poll(() => activeTools(page)).toEqual([...BASE_TOOLS, 'control_microduck_physical_simulation']);
  const task = await callTool(page, 'describe_robobuddy_task');
  expect(task).toMatchObject({ simulationMode: 'physical_mujoco', hardwareValidated: false });
  const schema = 'robobuddy.microduck.physical.v1';
  const advance = await callTool(page, 'control_microduck_physical_simulation', { schema_version: schema, command: 'advance', advance_seconds: 0.02 });
  expect(advance).toMatchObject({ ok: true, command: 'advance', executedTicks: 1 });
  const invalid = await callTool(page, 'control_microduck_physical_simulation', { schema_version: schema, command: 'set_command', request: { vx: null } });
  expect(invalid).toMatchObject({ ok: false, error: { code: 'INVALID_ARGUMENT' } });
  const cancelled = await callTool(page, 'control_microduck_physical_simulation', { schema_version: schema, command: 'advance', advance_seconds: 2 }, { aborted: true });
  expect(cancelled).toMatchObject({ ok: false, error: { code: 'OPERATION_CANCELLED' } });
  await page.locator('#robotSelect').selectOption('openarm');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await expect.poll(() => activeTools(page)).toEqual(OPENARM_TOOLS);
  expect(await page.evaluate(() => window.__webMcpRegistrations.filter(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted).length)).toBe(0);
});

'''+s[b:]
s=s.replace("toContainText('Loading local MicroDuck')", "toContainText('Loading MicroDuck MuJoCo physical')")
s=s.replace("tool.name === 'control_microduck_simulation'", "tool.name === 'control_microduck_physical_simulation'")
p.write_text(s)
p=Path('.github/workflows/validate.yml');s=p.read_text()
s=s.replace('          node --check tests/microduck-control-deck-browser-smoke.spec.mjs\n','')
a=s.index('      - name: MicroDuck starter Chromium regression');b=s.index('      - name: Preserve browser failure evidence',a)
s=s[:a]+'''      - name: Chromium runtime, physical MicroDuck presentation and full source-plant replay
        run: npm run test:browser -- --reporter=line
'''+s[b:];p.write_text(s)
p=Path('.github/workflows/microduck-phase5c-validate.yml');s=p.read_text()
s=replace(s, '      - name: MicroDuck pinned asset gate', '''      - name: MicroDuck complete-body presentation contract
        run: node tests/physics/microduck-presentation-core.mjs

      - name: MicroDuck pinned asset gate''')
s=replace(s, 'tests/microduck-interfaces-browser.spec.mjs --reporter=line', 'tests/microduck-interfaces-browser.spec.mjs tests/microduck-presentation-browser.spec.mjs tests/microduck-policy-parity.spec.mjs --reporter=line')
p.write_text(s)
p=Path('README.md');s=p.read_text()
a=s.index('Both MicroDuck workspaces are entered by name');b=s.index(' See `docs/physics/microduck-provenance.md`.',a)
s=s[:a]+'''MicroDuck now exposes only three physical tasks: locomotion (default), ground contact, and ball kick. The legacy articulated policy demonstrator is retired. Rendered bodies use one authoritative MuJoCo body-pose snapshot, and no cosmetic roller attachments are created in physical tasks.'''+s[b:]
s=s.replace('LeKiwi, MicroDuck, Unitree G1, Panda, and ASIMOV remain separate later robot-specific migration work.', 'LeKiwi and MicroDuck have separate physical workspaces. Unitree G1, Panda, and ASIMOV remain separate later robot-specific migration work.')
p.write_text(s)
Path('docs/physics/microduck-presentation.md').write_text('''# MicroDuck physical presentation and task retirement

## Corrected frames

The source DUCK v1 hierarchy stores the initial trunk world transform, including 0.12 m height. The physical renderer must replace that transform, not add it to the observed trunk pose. The model-to-view basis is (x, y, z) -> (x, z, -y), with metres scaled to millimetres. Zero quaternion components, including w=0 in a half-turn, are valid.

The model descriptors publish every one of the 15 source robot bodies. The renderer consumes their MuJoCo xpos/xquat world-pose snapshot and derives parent-relative presentation transforms. It does not recompute joint FK from qpos. MuJoCo's solved kinematic/contact fields after mj_step describe its force-evaluation stage, whereas qpos has already been integrated; those stages must not be mixed. Rendering performs no new mj_forward or mj_step and does not change controller/sensor timing, BAM forces, contacts, or any state. See https://mujoco.readthedocs.io/en/stable/computation/index.html#consistency-in-mjdata.

A missing, nonfinite, or degenerate body pose rejects the whole visual update before mutation. The source visual mesh bytes, inertials, collision meshes and physics thresholds are unchanged. The official soles remain; the local cosmetic passive-roller assemblies are not constructed in physical workspaces. The source reset has a small genuine sole clearance; ordinary gravity/contact closes it when simulation is advanced. No floor snap, hidden settle, or synthetic support is introduced.

## Supported learner tasks

Physical MicroDuck Locomotion is the default. Physical MicroDuck Ground Contact and Physical MicroDuck Ball Kick retain their task-specific plants. The articulated policy demonstrator is removed from the selector, scenario resolver and SimulatorHost routing. A saved old task selection migrates to the physical default at UI load; a direct request for the retired scenario is rejected. Its optional roller/audio/peripheral features are not advertised as physical features.

## Regression scope

The presentation browser suite checks all 15 rendered body positions/orientations against MuJoCo at reset, stationary contact and walking, at unchanged 1 micrometre / 10 microradian tolerances. It checks all three tasks, the absence of cosmetic rollers, all 58 official parts, sole clearance, real named floor contact, non-snapping half-turns/tilts, and atomic failure on invalid snapshots. The existing native BAM and contact-based physical gates are retained unchanged. Legacy-demonstrator-only UI browser suites are removed because that product surface is retired, not skipped to disguise a physical failure; their independent ONNX CPU/browser parity check remains active alongside physical Python, WebMCP and cancellation suites. Low-level legacy module unit tests are retained as isolation/provenance tests while shared assets remain in use.

This is simulator/presentation verification, not assembled-hardware calibration.
''')
for name in ['docs/microduck-simulator.md', 'docs/physics/microduck-simulator.md']:
    p=Path(name);p.write_text('> Historical demonstrator documentation. The articulated policy demonstrator is retired from the IDE. Current tasks are physical locomotion, ground contact and ball kick; see `docs/physics/microduck-presentation.md`. Features described below are not advertised physical capabilities.\n\n'+p.read_text())
print('Coherent MicroDuck rendering and physical-only task retirement applied')
