from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def replace(path, old, new):
    p = ROOT / path
    s = p.read_text()
    n = s.count(old)
    if n != 1:
        raise RuntimeError(f'{path}: expected one patch anchor, found {n}: {old[:100]}')
    p.write_text(s.replace(old, new, 1))

# A capability is not necessarily an executable controller command.
p = 'src/physics/microduck-capabilities.js'
replace(p, 'export function microduckCapability(id) {\n  return MICRODUCK_CAPABILITY_AUDIT.find((item) => item.id === id) || null;\n}', '''// Explicit controller commands, separate from the capability audit above.
export const MICRODUCK_SKILL_CAPABILITIES = Object.freeze({
  sit: 'sit_stand', stand_up: 'sit_stand', ground_pick: 'ground_pick',
  kick_left: 'kick_left', kick_right: 'kick_right', roulade: 'roulade',
});
export const MICRODUCK_PHYSICAL_SKILL_IDS = Object.freeze(Object.keys(MICRODUCK_SKILL_CAPABILITIES));

export function microduckCapability(id) {
  const capabilityId = Object.hasOwn(MICRODUCK_SKILL_CAPABILITIES, id) ? MICRODUCK_SKILL_CAPABILITIES[id] : id;
  return MICRODUCK_CAPABILITY_AUDIT.find((item) => item.id === capabilityId) || null;
}''')

p = 'src/physics/microduck-physical-simulator.js'
replace(p, '  microduckRequiredPackageKeys,', '  microduckRequiredPackageKeys, MICRODUCK_PHYSICAL_SKILL_IDS,')
replace(p, '''  /** Latch a bounded command. Accepting it is not achieving it. */
  setCommand(requested = {}) {''', '''  /** Validate without mutating command, controller or evaluator state. */
  validateCommand(requested = {}) {''')
replace(p, '''    this.requested = command;
    this.requestedLimitedBy = limitedBy;
    this.evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: command.twist });''', '''    return { command, limitedBy };
  }

  /** Latch a bounded command. Accepting it is not achieving it. */
  setCommand(requested = {}) {
    const { command, limitedBy } = this.validateCommand(requested);
    this.requested = command;
    this.requestedLimitedBy = limitedBy;
    this.evaluator = new MicroDuckPhysicalEvaluator({ commandedTwist: command.twist });''')
replace(p, '''  requestSkill(skill) {
    this.#assertLoaded();
    const capabilityId = skill === 'sit' || skill === 'stand_up' ? 'sit_stand' : skill;
    const capability = microduckCapability(capabilityId) || MICRODUCK_CAPABILITY_AUDIT.find((item) => item.physicalPolicy === skill);''', '''  validateSkillRequest(skill) {
    this.#assertLoaded();
    const capability = microduckCapability(skill);''')
replace(p, '''    const started = this.controller.requestSkill(skill);
    if (!started) return { accepted: false, status: 'unsupported', capability: skill, reason: `${skill} is not a physical skill of this workspace` };
    return { accepted: true, status: 'running', capability: capability?.id ?? skill };''', '''    if (!MICRODUCK_PHYSICAL_SKILL_IDS.includes(skill) || !this.controller.hasPolicy(capability?.physicalPolicy)) {
      return { accepted: false, status: 'unsupported', capability: skill, reason: `${skill} is not a physical skill command; use sit or stand_up for the sit_stand capability` };
    }
    return { accepted: true, status: 'available', capability: capability.id };
  }

  requestSkill(skill) {
    const result = this.validateSkillRequest(skill);
    if (!result.accepted) return result;
    if (!this.controller.requestSkill(skill)) return { accepted: false, status: 'unsupported', capability: skill, reason: 'The controller refused the skill' };
    return { ...result, status: 'running' };''')

p = 'src/app-v2.js'
replace(p, "import { LivePythonBridge } from './runtime/live-python-bridge.js';", "import { LivePythonBridge } from './runtime/live-python-bridge.js';\nimport { MicroDuckPhysicalBridge } from './runtime/microduck-physical-bridge.js';")
replace(p, "        return new LivePythonBridge(session);", "        return this.sim.profileId === 'microduck'\n          ? new MicroDuckPhysicalBridge(this.sim.backend)\n          : new LivePythonBridge(session);")

p = 'src/webmcp/microduck-physical-control.js'
replace(p, 'MICRODUCK_CAPABILITY_STATUS, microduckCapability', 'MICRODUCK_CAPABILITY_STATUS, MICRODUCK_PHYSICAL_SKILL_IDS, microduckCapability')
replace(p, '''const PHYSICAL_SKILLS = Object.freeze(
  MICRODUCK_CAPABILITY_AUDIT
    .filter((item) => item.physicalPolicy && !['stand', 'walk', 'recovery'].includes(item.id))
    .map((item) => item.id),
);''', 'const PHYSICAL_SKILLS = MICRODUCK_PHYSICAL_SKILL_IDS;')
replace(p, "    type: 'object',\n    oneOf: [", "    type: 'object',\n    // Root closure sees only sibling properties, not properties inside oneOf.\n    // Each command branch below still rejects fields from all other branches.\n    properties: { schema_version: version, command: { type: 'string' }, request: {}, skill: {}, advance_seconds: {}, perturbation: {}, settle_seconds: {} },\n    oneOf: [")
replace(p, "description: 'a capability id; unsupported ids are rejected explicitly rather than routed elsewhere'", "description: 'an explicit controller command (sit or stand_up for sit/stand); unsupported roller ids are rejected without a legacy fallback'")
replace(p, '    if (value == null) {', '    if (value === undefined) {')
replace(p, "    const seconds = Number(value);\n    if (!Number.isFinite(seconds)", "    const seconds = value;\n    if (typeof seconds !== 'number' || !Number.isFinite(seconds)")
replace(p, "    if (typeof input.skill !== 'string' || !input.skill) invalid('skill must be a capability id.');", "    if (![...PHYSICAL_SKILLS, ...UNSUPPORTED_SKILLS].includes(input.skill)) invalid('skill must be a supported command id; use sit or stand_up for sit/stand.');")
replace(p, "    const value = Number(raw);\n    if (!Number.isFinite(value)) invalid(`${key} must be finite.`);\n    request[key] = value;", "    const value = raw;\n    if (typeof value !== 'number' || !Number.isFinite(value)) invalid(`${key} must be a finite number.`);\n    const ranges = { vx: L.vxMS, vy: L.vyMS, vyaw: L.vyawRadS, neckPitch: L.neckPitchRad, headPitch: L.headPitchRad, headYaw: L.headYawRad, headRoll: L.headRollRad, bodyZ: L.bodyZM, bodyRoll: L.bodyRollRad, bodyPitch: L.bodyPitchRad };\n    const [minimum, maximum] = ranges[key];\n    if (value < minimum || value > maximum) invalid(`${key} must be between ${minimum} and ${maximum}.`);\n    request[key] = value;")
replace(p, '  const simulator = facade.app.sim;', '  const simulator = facade.app.sim.backend;')
replace(p, '    assertCurrent(facade, baseline, expectedEpoch, signal);\n\n    if (parsed.command', "    assertCurrent(facade, baseline, expectedEpoch, signal);\n    if (typeof simulator?.setCommand !== 'function' || typeof simulator?.advanceSeconds !== 'function') {\n      throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The active MicroDuck physical command backend is unavailable.');\n    }\n\n    if (parsed.command")
replace(p, '      const run = await simulator.advanceSeconds(parsed.advanceSeconds);', "      const run = parsed.advanceSeconds === 0\n        ? { executedTicks: 0, completed: true, cancelled: false, simulatedSeconds: 0 }\n        : await simulator.advanceSeconds(parsed.advanceSeconds);")

p = 'src/runtime/microduck-physical-bridge.js'
replace(p, 'MICRODUCK_CAPABILITY_AUDIT, microduckCapability', 'MICRODUCK_CAPABILITY_AUDIT, MICRODUCK_PHYSICAL_SKILL_IDS, MICRODUCK_UNSUPPORTED_CAPABILITIES, microduckCapability')
replace(p, 'const SKILL_IDS = Object.freeze(MICRODUCK_CAPABILITY_AUDIT.map((item) => item.id));', 'const SKILL_IDS = Object.freeze([...MICRODUCK_PHYSICAL_SKILL_IDS, ...MICRODUCK_UNSUPPORTED_CAPABILITIES]);')
replace(p, '  const number = Number(value);', '  const number = value;')
replace(p, '  if (!Number.isFinite(number) || number < 0)', "  if (typeof number !== 'number' || !Number.isFinite(number) || number < 0)")
replace(p, '      commandFields: [...COMMAND_FIELDS],', '      commandFields: [...COMMAND_FIELDS],\n      skillCommands: [...MICRODUCK_PHYSICAL_SKILL_IDS],')
replace(p, '''        skill = String(raw);
        if (!SKILL_IDS.includes(skill)) throw liveError('INVALID_ARGUMENT', `Unknown MicroDuck capability: ${skill}`);''', '''        skill = raw;
        if (typeof skill !== 'string' || !SKILL_IDS.includes(skill)) throw liveError('INVALID_ARGUMENT', 'Unknown MicroDuck skill command; use sit or stand_up for sit/stand');''')
replace(p, '''      const value = Number(raw);
      if (!Number.isFinite(value)) throw liveError('INVALID_ARGUMENT', `${key} must be finite`);''', '''      const value = raw;
      if (typeof value !== 'number' || !Number.isFinite(value)) throw liveError('INVALID_ARGUMENT', `${key} must be a finite number`);''')
replace(p, '''    const accepted = Object.keys(request).length ? this.simulator.setCommand(request) : null;
    let skillResult = null;
    if (skill) {
      skillResult = this.simulator.requestSkill(skill);''', '''    // Validate the entire combined action before latching either part. A rejected
    // skill must not leave its accompanying velocity command running.
    if (Object.keys(request).length) this.simulator.validateCommand(request);
    let skillResult = null;
    if (skill) {
      skillResult = this.simulator.validateSkillRequest(skill);''')
replace(p, '''    let run = null;
    if (advanceSeconds > 0) run = await this.simulator.advanceSeconds(advanceSeconds);''', '''    const accepted = Object.keys(request).length ? this.simulator.setCommand(request) : null;
    if (skill) skillResult = this.simulator.requestSkill(skill);
    let run = null;
    if (advanceSeconds > 0) run = await this.simulator.advanceSeconds(advanceSeconds);
    this.#assertOwner();''')
replace(p, '''    const timeout = Math.min(finiteNonNegative(timeoutSeconds, 'timeout_seconds'), MICRODUCK_MAX_ADVANCE_SECONDS);
    const period = Math.max(MICRODUCK_CONTROL_INTERVAL_SECONDS, finiteNonNegative(controllerPeriodSeconds, 'controller_period_seconds'));
    const upright = goal?.upright !== false;
    const deadlineTicks = Math.max(1, Math.round(timeout / period));
    let reached = false;
    for (let tick = 0; tick < deadlineTicks; tick += 1) {
      await this.simulator.advanceSeconds(period);
      const report = this.simulator.report();
      if (upright && report?.upright) { reached = true; break; }
      if (this.simulator.cancelled) break;
    }
    return { ...this.getObservation(), reached, timedOut: !reached, timeoutSeconds: timeout };''', '''    if (!goal || typeof goal !== 'object' || Array.isArray(goal) || Object.keys(goal).some((key) => key !== 'upright')
      || (goal.upright !== undefined && typeof goal.upright !== 'boolean')) {
      throw liveError('INVALID_ARGUMENT', 'MicroDuck wait_for_goal supports only an upright boolean goal');
    }
    const timeout = Math.min(finiteNonNegative(timeoutSeconds, 'timeout_seconds'), MICRODUCK_MAX_ADVANCE_SECONDS);
    const period = finiteNonNegative(controllerPeriodSeconds, 'controller_period_seconds');
    const dt = MICRODUCK_CONTROL_INTERVAL_SECONDS;
    // Use whole controller ticks, flooring the budget so a wait never exceeds its timeout.
    const budgetTicks = Math.floor(timeout / dt + 1e-10);
    const periodTicks = Math.max(1, Math.min(Math.floor(MICRODUCK_MAX_ADVANCE_SECONDS / dt), Math.floor(period / dt + 1e-10)));
    const desired = goal.upright !== false;
    const goalReached = () => this.simulator.report()?.upright === desired;
    let reached = goalReached();
    let elapsedTicks = 0;
    let interrupted = false;
    while (!reached && elapsedTicks < budgetTicks) {
      this.#assertOwner();
      const ticks = Math.min(periodTicks, budgetTicks - elapsedTicks);
      const run = await this.simulator.advanceSeconds(ticks * dt);
      this.#assertOwner();
      elapsedTicks += run.executedTicks;
      reached = goalReached();
      if (!run.completed) { interrupted = true; break; }
    }
    return { ...this.getObservation(), reached, timedOut: !reached && !interrupted,
      interrupted, timeoutSeconds: timeout, simulatedSeconds: elapsedTicks * dt };''')
replace(p, '''    if (authority.sessionId !== this.owner.sessionId || authority.sceneRevision !== this.owner.sceneRevision || authority.robotId !== this.owner.robotId) {''', '''    if (authority.sessionId !== this.owner.sessionId || authority.sceneRevision !== this.owner.sceneRevision || authority.robotId !== this.owner.robotId
      || authority.epoch !== this.owner.epoch || this.simulator.runEpoch !== this.owner.runEpoch) {''')

p = 'src/task-workspace.js'
replace(p, 'function physicalWorkspace(profileId, scenario) {', '''function microduckPhysicalWorkspace(scenario) {
  const commands = scenario.id === 'microduck-physical-kick'
    ? [{ label: 'Request a right-foot kick; observe contact and ball motion', action: { skill: 'kick_right' }, duration_seconds: 3 }]
    : scenario.id === 'microduck-physical-groundcontact'
    ? [{ label: 'Stand through the deployed policy', action: { vx: 0 }, duration_seconds: 1 },
       { label: 'Request sitting', action: { skill: 'sit' }, duration_seconds: 1 },
       { label: 'Request rising (not a fall-recovery reset)', action: { skill: 'stand_up' }, duration_seconds: 1 }]
    : [{ label: 'Request forward walking through foot-floor contact', action: { vx: 0.35 }, duration_seconds: 2 }];
  const stages = commands.map((stage, index) => ({ index: index + 1, ...stage }));
  const main = [
    '# MicroDuck physical MuJoCo workspace. No hardware transport is opened.',
    '# Commands are requests to the deployed policy; only actual is measured simulation state.',
    '# A kick can miss. A posture request can fail. Neither is manufactured into success.',
    'from robobuddy.sim import connect',
    'from robot_config import ROBOT_ID, CONTROL_INTERVAL_S',
    'from trajectories import STAGES',
    '',
    'robot = await connect(ROBOT_ID)',
    'try:',
    '    for stage in STAGES:',
    '        accepted = await robot.send_action(stage["action"])',
    '        remaining_ticks = int(round(stage["duration_seconds"] / CONTROL_INTERVAL_S))',
    '        while remaining_ticks > 0:',
    '            ticks = min(10, remaining_ticks)',
    '            observation = await robot.advance(ticks * CONTROL_INTERVAL_S)',
    '            remaining_ticks -= ticks',
    '        actual = (await robot.get_observation())["actual"]',
    '        print(stage["label"], "actual trunk (m)", actual["trunkPositionM"], "tilt (deg)", actual["trunkTiltDeg"])',
    '    final = await robot.get_observation()',
    '    print("MicroDuck physical observation", final["actual"])',
    '    print("Measured task report", final["taskEvaluation"])',
    'finally:',
    '    await robot.disconnect()',
    '',
  ].join('\\n');
  return {
    'main.py': main,
    'trajectories.py': `# Explicit policy commands, not direct joint or free-body state writes.\\n# Durations use simulation time. Edit these commands for the selected collision plant.\\nSTAGES = ${py(stages)}\\n`,
    'robot_config.py': `# Browser simulation only. No serial, CAN or network robot connection.\\nROBOT_ID = ${JSON.stringify(scenario.robotId)}\\nPHYSICAL_API_VERSION = "robobuddy.sim.v1"\\nCONTROL_INTERVAL_S = ${scenario.controller.controlIntervalSeconds}\\n`,
    'workcell.py': `# Read-only physical workspace metadata, not installed-hardware calibration.\\nWORKCELL = ${py({ scenario_id: scenario.id, physical_scene_id: scenario.physicalSceneId, model_package: scenario.modelPackage, capabilities: scenario.capabilities, limitations: scenario.limitations })}\\n`,
  };
}

function physicalWorkspace(profileId, scenario) {
  if (profileId === 'microduck') return microduckPhysicalWorkspace(scenario);''')
replace('src/task-catalog.js', '    workspaceRevision: physicalScene.revision, id, title, brief, robotId: modelPackage.robotId,', "    workspaceRevision: `${physicalScene.revision}-public-interfaces-v2`, id, title, brief, robotId: modelPackage.robotId,")
replace('playwright.config.mjs', '|microduck-phase5c-browser)', '|microduck-phase5c-browser|microduck-interfaces-browser)')
replace('.github/workflows/microduck-phase5c-validate.yml', '      - name: MicroDuck pinned asset gate', '''      - name: MicroDuck public command, bridge and starter regressions
        run: |
          node tests/physics/microduck-interfaces-core.mjs /tmp/microduck-interfaces.json
          python -m pip install --disable-pip-version-check 'jsonschema==4.23.0'
          python tests/validate_microduck_interfaces.py /tmp/microduck-interfaces.json

      - name: MicroDuck pinned asset gate''')
replace('.github/workflows/microduck-phase5c-validate.yml', 'npx playwright test tests/microduck-phase5c-browser.spec.mjs --reporter=line', 'npx playwright test tests/microduck-phase5c-browser.spec.mjs tests/microduck-interfaces-browser.spec.mjs --reporter=line')
print('MicroDuck public-interface fixes applied; physical plant and policy parameters unchanged.')
