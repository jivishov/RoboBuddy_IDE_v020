from pathlib import Path


def replace(text, old, new, count=1):
    assert text.count(old) == count, (old[:80], text.count(old), count)
    return text.replace(old, new)


p = Path('src/microduck/rig-adapter.js')
s = p.read_text()
s = replace(s, '  static async load() {', '  static async load(options = {}) {')
s = replace(s, 'return new MicroDuckRigAdapter(data, visual);', 'return new MicroDuckRigAdapter(data, visual, options);')
s = replace(s, '  constructor(data, visual) {', '  constructor(data, visual, { includeConfiguredRollers = true } = {}) {')
s = replace(s, '    this.visual = visual;', '    this.visual = visual;\n    this.includeConfiguredRollers = includeConfiguredRollers;')
s = replace(s, '    this.buildConfiguredAttachments();\n    this.applyState({});', "    this.buildConfiguredAttachments();\n    this.setVariant('walking');\n    this.applyState({});")
s = replace(s, '    for (const roller of this.data.configuredAttachments?.rollers || []) {', '    for (const roller of (this.includeConfiguredRollers ? this.data.configuredAttachments?.rollers || [] : [])) {')
old = '''    this.root.position.set((Number(position[0]) || 0) * 1000, (Number(position[2]) || 0) * 1000, -(Number(position[1]) || 0) * 1000);
    this.root.quaternion.set(Number(quaternionWxyz[1]) || 0, Number(quaternionWxyz[3]) || 0, -(Number(quaternionWxyz[2]) || 0), Number(quaternionWxyz[0]) || 1).normalize();
    this.root.updateWorldMatrix(true, true);'''
new = '''    // DUCK stores the trunk's initial WORLD transform (including its 120 mm height).
    // MuJoCo now supplies that world transform. Keep only the descendant local frames;
    // composing both world placements would lift every rendered contact by another 120 mm.
    const trunk = this.bodies.get('trunk_base');
    if (!trunk || trunk.parent !== this.modelRoot) throw new Error('MicroDuck physical visual requires a trunk_base root.');
    trunk.position.set(0, 0, 0);
    trunk.quaternion.identity();
    this.root.position.set((Number(position[0]) || 0) * 1000, (Number(position[2]) || 0) * 1000, -(Number(position[1]) || 0) * 1000);
    const q = quaternionFromWxyz(quaternionWxyz);
    // Basis change R_view = C R_model C^-1, C: (x,y,z) -> (x,z,-y).
    // A valid half-turn has w=0; do not replace that zero with the identity's w=1.
    this.root.quaternion.set(q.x, q.z, -q.y, q.w);
    this.root.updateWorldMatrix(true, true);'''
s = replace(s, old, new)
s = replace(s, '  applyRootPose(position = [0, 0, 0], quaternionWxyz = [1, 0, 0, 0]) {', '''  applyRootPose(position = [0, 0, 0], quaternionWxyz = [1, 0, 0, 0]) {
    this.visual.bodies.forEach((body, index) => {
      if (body.parentIndex >= 0) return;
      this.bodyList[index].position.fromArray(body.positionM);
      this.bodyList[index].quaternion.copy(quaternionFromWxyz(body.quaternionWxyz));
    });''')
p.write_text(s)
p = Path('src/physics/microduck-physical-simulator.js')
s = replace(p.read_text(), 'this.rig = await rigModule.MicroDuckRigAdapter.load();', 'this.rig = await rigModule.MicroDuckRigAdapter.load({ includeConfiguredRollers: false });')
s = replace(s, '    this.resize();\n  }\n\n  #ensureBallMesh', "    this.rig.setVariant?.('walking');\n    this.resize();\n  }\n\n  #ensureBallMesh")
s = replace(s, "      rootTransformSource: 'observed MuJoCo trunk_base free-body pose',", "      rootTransformSource: 'observed MuJoCo trunk_base free-body pose',\n      bakedWorldRootApplied: false,\n      configuredRollers: false,")
p.write_text(s)
p = Path('src/task-catalog.js')
s = p.read_text(); a = s.index('const MICRODUCK_SCENARIO ='); b = s.index('const cache =', a)
s = s[:a] + "// The legacy policy demonstrator was retired at the user's request. Physical locomotion\n// is the default; each remaining task keeps its own collision plant and command surface.\nconst MICRODUCK_TASKS = MICRODUCK_PHYSICAL_TASKS;\n\n" + s[b:]
s = replace(s, '  const legacySo101 =', "  if (profileId === 'microduck' && taskId && !MICRODUCK_TASKS.some((item) => item.id === taskId)) return null;\n  const legacySo101 =")
s = replace(s, "  if (descriptor.simulationMode === 'policy_sim') return structuredClone(MICRODUCK_SCENARIO);\n", '')
p.write_text(s)
p = Path('src/simulator-host.js')
s = replace(p.read_text(), "import { MicroDuckPolicySimulator } from './microduck/policy-simulator.js';\n", '')
s = replace(s, '    microduckFactory = (target) => new MicroDuckPolicySimulator(target, { externalClock: true }),\n', '')
s = replace(s, '    this.microduckFactory = microduckFactory;\n', '')
s = replace(s, '    const epoch = ++this.epoch;', "    if (profileId === 'microduck' && scenario?.simulationMode !== 'physical_mujoco') {\n      throw new Error('MicroDuck supports physical tasks only; the legacy policy demonstrator has been retired.');\n    }\n    const epoch = ++this.epoch;")
s = replace(s, "      : profileId === 'microduck'\n      ? this.microduckFactory(this.canvas)\n", '')
p.write_text(s)
p = Path('src/profiles.js')
s = replace(p.read_text(), "label:'MicroDuck Policy Demonstrator'", "label:'MicroDuck Physical'")
s = replace(s, "driver:'Main-thread 50 Hz browser policy simulator', transport:'none — local browser assets', simulationMode:'policy_sim'", "driver:'50 Hz controller · worker-backed MuJoCo', transport:'none — local browser simulation', simulationMode:'physical_mujoco'")
s = replace(s, 'Configured lower-bill movement, rollers, visual floor alignment, collisions, contacts and dynamics are approximations; no RL-model, locomotion or hardware parity is claimed.', 'Source-derived contact plants and bounded controllers drive the physical tasks. The renderer consumes observed state; no cosmetic rollers or floor snapping are used. Numerical verification is not hardware calibration.')
a = s.index("    task:Object.freeze({title:'Approximate browser policy simulation'"); b = s.index('\n', a)
s = s[:a] + "    task:Object.freeze({title:'Physical locomotion, ground contact and ball kick', steps:['Select the task-specific physical plant','Run bounded async Python or WebMCP commands','Inspect actual MuJoCo foot-floor and foot-ball contacts','Compare the requested command with measured motion'], limitations:'Source-derived simulation models, not a calibrated hardware twin. Roller modes have no matched physical plant and are unsupported. A requested skill is not evidence of success.'})," + s[b:]
a = s.index("  if (profile?.simulationMode === 'policy_sim') return"); b = s.index('\n', a)
s = s[:a] + "  if (profileId === 'microduck') return 'MicroDuck physical tasks use one browser MuJoCo authority, source-derived collision plants and bounded policy control. Rendered body poses follow measured simulation state. Roller modes are unsupported; simulator verification is not hardware calibration.';" + s[b:]
p.write_text(s)
p = Path('playwright.config.mjs')
s = replace(p.read_text(), '|microduck-interfaces-browser)', '|microduck-interfaces-browser|microduck-presentation-browser)')
p.write_text(s)
