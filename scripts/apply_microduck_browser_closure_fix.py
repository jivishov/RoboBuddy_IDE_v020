#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# 1) Effective BAM gain diagnostics: when motor actuation is disabled, the physical authority
# must report zero effective firmware gain even though the controller continues requesting a
# scheduled gain and joint targets.
worker_path = ROOT / 'src/physics/microduck-mujoco-worker.js'
worker = worker_path.read_text()
old = "      smallSignalKpNmRad: microDuckBamSmallSignalKpNmRad(appliedFirmwareGain, effectiveVinV),\n"
new = "      smallSignalKpNmRad: microDuckBamSmallSignalKpNmRad(actuationEnabled ? appliedFirmwareGain : 0, effectiveVinV),\n"
if worker.count(old) != 1:
    raise SystemExit(f'worker small-signal gain marker drifted: {worker.count(old)}')
worker = worker.replace(old, new, 1)
old = "    actuationEnabled, firmwareGain: appliedFirmwareGain,\n"
new = "    actuationEnabled, firmwareGain: actuationEnabled ? appliedFirmwareGain : 0,\n"
if worker.count(old) != 1:
    raise SystemExit(f'worker effective gain marker drifted: {worker.count(old)}')
worker = worker.replace(old, new, 1)
worker_path.write_text(worker)

# 2) Alias plant safety. `sit` and `stand_up` are controller commands for the audited
# `sit_stand` capability; they must not bypass task-specific collision-plant checks.
sim_path = ROOT / 'src/physics/microduck-physical-simulator.js'
sim = sim_path.read_text()
old = """    const capability = microduckCapability(skill) || MICRODUCK_CAPABILITY_AUDIT.find((item) => item.physicalPolicy === skill);\n    if (capability && !capability.physicalPolicy) {\n"""
new = """    const capabilityId = skill === 'sit' || skill === 'stand_up' ? 'sit_stand' : skill;\n    const capability = microduckCapability(capabilityId) || MICRODUCK_CAPABILITY_AUDIT.find((item) => item.physicalPolicy === skill);\n    if (capability && !capability.physicalPolicy) {\n"""
if sim.count(old) != 1:
    raise SystemExit(f'simulator capability marker drifted: {sim.count(old)}')
sim = sim.replace(old, new, 1)
old = """          // The servo gain the authority actually applied and the force it produced. The\n          // controller view above reports the gain it ASKED for; this is what physics used.\n          // Both are published because a torque-off is only believable if the force reads zero.\n          firmwareGain: observation.firmwareGain ?? null,\n          appliedServoKp: observation.appliedServoKp ?? null,\n"""
new = """          // The effective BAM firmware gain and actual actuator force come from the physical\n          // authority. The controller view above reports the scheduled gain it ASKED for.\n          // `appliedServoKp` is retained only as the source-XML equivalent compatibility\n          // diagnostic; BAM torque physics does not use that fallback PD stiffness.\n          firmwareGain: observation.firmwareGain ?? null,\n          appliedServoKp: observation.appliedServoKp ?? null,\n"""
if sim.count(old) != 1:
    raise SystemExit(f'simulator diagnostic-comment marker drifted: {sim.count(old)}')
sim = sim.replace(old, new, 1)
sim_path.write_text(sim)

# 3) Browser fidelity journey: a kick request on the walking plant must be refused as
# wrong-plant, then the kick is exercised after an explicit kick-package load. Recovery must
# likewise use the broad all-collision plant and the deployed stand policy, not the sit/stand
# posture-transition alias.
browser_path = ROOT / 'tests/microduck-phase5c-browser.spec.mjs'
browser = browser_path.read_text()
old = """  expect(capabilities.rollerCrouch.accepted).toBe(false);\n  expect(capabilities.kickRight.accepted).toBe(true);\n  expect(capabilities.table.find((item) => item.id === 'roller').physical).toBe(false);\n"""
new = """  expect(capabilities.rollerCrouch.accepted).toBe(false);\n  expect(capabilities.kickRight.accepted).toBe(false);\n  expect(capabilities.kickRight.status).toBe('wrong-plant');\n  expect(capabilities.kickRight.requiredPackageKeys).toContain('kick');\n  expect(capabilities.table.find((item) => item.id === 'roller').physical).toBe(false);\n"""
if browser.count(old) != 1:
    raise SystemExit(f'browser wrong-plant expectation marker drifted: {browser.count(old)}')
browser = browser.replace(old, new, 1)
old = """  // A torque-off must produce NO actuator force. These are position actuators, so zeroing\n  // ctrl instead would command every joint to 0 rad at full strength - actuation toward a\n  // straight-legged pose, not its absence. Zero here is what makes this a negative control\n  // rather than a differently-posed positive one, and it is the only assertion that proves\n  // the zeroed servo gain really took effect inside the WASM model.\n"""
new = """  // A torque-off must produce NO actuator force. The controller still emits position\n  // targets, but the BAM motor law is disabled and the MuJoCo motor torque is zero. Reporting\n  // zero effective firmware gain and zero force makes this a true negative control rather\n  // than a differently-commanded positive run.\n"""
if browser.count(old) != 1:
    raise SystemExit(f'browser torque-off comment marker drifted: {browser.count(old)}')
browser = browser.replace(old, new, 1)
old = """  const recovery = await page.evaluate(async () => {\n    const sim = window.__robobuddyCi.app.sim.backend;\n    await sim.reset();\n    await sim.applyPerturbation('face_down');\n    await sim.settle(1.5);\n    const settled = sim.getState().actual;\n    sim.requestSkill('stand_up');\n    for (let i = 0; i < 4; i += 1) await sim.advanceSeconds(2);\n    const recovered = sim.report();\n\n    await sim.reset();\n    await sim.applyPerturbation('face_down');\n    await sim.settle(1.5);\n    await sim.setActuationEnabled(false);\n    for (let i = 0; i < 4; i += 1) await sim.advanceSeconds(2);\n    const failed = sim.report();\n    return { settled, recovered, failed };\n  });\n"""
new = """  const recovery = await page.evaluate(async () => {\n    const sim = window.__robobuddyCi.app.sim.backend;\n    await sim.load('groundContact');\n    await sim.reset();\n    await sim.applyPerturbation('face_down');\n    await sim.settle(1.5);\n    const settled = sim.getState().actual;\n    // Zero velocity on the broad ground-contact plant selects the deployed stand policy,\n    // which is the pinned fall-recovery reference. Do not substitute the sit/stand rise alias.\n    for (let i = 0; i < 4; i += 1) await sim.advanceSeconds(2);\n    const recovered = sim.report();\n    const recoveryPolicy = sim.getState().controller?.policyId ?? null;\n\n    await sim.reset();\n    await sim.applyPerturbation('face_down');\n    await sim.settle(1.5);\n    await sim.setActuationEnabled(false);\n    for (let i = 0; i < 4; i += 1) await sim.advanceSeconds(2);\n    const failed = sim.report();\n    return { settled, recovered, recoveryPolicy, failed };\n  });\n"""
if browser.count(old) != 1:
    raise SystemExit(f'browser recovery block marker drifted: {browser.count(old)}')
browser = browser.replace(old, new, 1)
old = """  // Physical recovery, through contact.\n  expect(recovery.recovered.upright).toBe(true);\n"""
new = """  // Physical recovery, through contact, using the pinned stand-policy route.\n  expect(recovery.recoveryPolicy).toBe('stand');\n  expect(recovery.recovered.upright).toBe(true);\n"""
if browser.count(old) != 1:
    raise SystemExit(f'browser recovery assertion marker drifted: {browser.count(old)}')
browser = browser.replace(old, new, 1)
browser_path.write_text(browser)

# 4) Lifecycle regression: controller aliases cannot bypass the collision-plant gate.
lifecycle_path = ROOT / 'tests/physics/microduck-lifecycle-core.mjs'
lifecycle = lifecycle_path.read_text()
old = """  const wrongPlant = sim.requestSkill('kick_right');\n  assert(wrongPlant.accepted === false && wrongPlant.status === 'wrong-plant', 'kick was not rejected on the walking collision plant');\n  assert(wrongPlant.requiredPackageKeys?.includes('kick'), 'wrong-plant response did not identify the required kick plant');\n  assert(worker.counts('command') === before, 'a wrong-plant capability reached the authority');\n  sim.dispose();\n"""
new = """  const wrongPlant = sim.requestSkill('kick_right');\n  assert(wrongPlant.accepted === false && wrongPlant.status === 'wrong-plant', 'kick was not rejected on the walking collision plant');\n  assert(wrongPlant.requiredPackageKeys?.includes('kick'), 'wrong-plant response did not identify the required kick plant');\n  for (const alias of ['sit', 'stand_up']) {\n    const aliasResult = sim.requestSkill(alias);\n    assert(aliasResult.accepted === false && aliasResult.status === 'wrong-plant', `${alias} bypassed the ground-contact plant gate`);\n    assert(aliasResult.requiredPackageKeys?.includes('groundContact'), `${alias} did not identify the required ground-contact plant`);\n  }\n  assert(worker.counts('command') === before, 'a wrong-plant capability reached the authority');\n  sim.dispose();\n"""
if lifecycle.count(old) != 1:
    raise SystemExit(f'lifecycle wrong-plant marker drifted: {lifecycle.count(old)}')
lifecycle = lifecycle.replace(old, new, 1)
lifecycle_path.write_text(lifecycle)

print('MicroDuck browser closure patch applied')
