#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
path = ROOT / 'tests' / 'lekiwi-presentation-browser.spec.mjs'
text = path.read_text()
pattern = re.compile(r"  const arm = await page\.evaluate\(async \(\) => \{[\s\S]*?  expect\(Math\.hypot\(\.\.\.wristAfter\.map\(\(value, index\) => value - wristBefore\[index\]\)\)\)\.toBeGreaterThan\(40\);\n")
replacement = r'''  const arm = await page.evaluate(async () => {
    const sim = window.__robobuddyCi.app.sim.backend;
    const { LEKIWI_ARM_POSES, LEKIWI_COURIER_CONTROLLER, LEKIWI_WORKCELL } = await import('/src/physics/lekiwi-scene.js');
    const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
    const wrap = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle));
    await sim.reset();
    await sim.advanceTime(0.2);

    // Reach the configured service stop through wheel actuation/contact before asking the arm to
    // reach the beaker.  The former test commanded the pickup pose from home, which is intentionally
    // outside arm reach and therefore could not test presentation alignment at the workcell.
    for (let i = 0; i < 120; i += 1) {
      const pose = sim.getBasePose();
      const dx = LEKIWI_WORKCELL.serviceStopXYM[0] - pose.xM;
      const dy = LEKIWI_WORKCELL.serviceStopXYM[1] - pose.yM;
      if (Math.hypot(dx, dy) < 0.012) break;
      const cos = Math.cos(pose.yawRad), sin = Math.sin(pose.yawRad);
      const forward = dx * cos + dy * sin;
      const left = -dx * sin + dy * cos;
      await sim.applyChassisVelocity({
        'x.vel': clamp(1.2 * forward, -0.15, 0.15),
        'y.vel': clamp(1.2 * left, -0.15, 0.15),
        'theta.vel': 0,
      }, { maxSteps: 4000, advanceSeconds: 0 });
      await sim.advanceTime(0.05);
    }
    await sim.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 2000, advanceSeconds: 0 });
    await sim.advanceTime(0.3);
    for (let i = 0; i < 100; i += 1) {
      const pose = sim.getBasePose();
      const yawError = wrap(LEKIWI_WORKCELL.serviceStopYawRad - pose.yawRad);
      if (Math.abs(yawError) < 0.04) break;
      await sim.applyChassisVelocity({
        'x.vel': 0, 'y.vel': 0,
        'theta.vel': clamp(yawError * 180 / Math.PI * 1.5, -45, 45),
      }, { maxSteps: 4000, advanceSeconds: 0 });
      await sim.advanceTime(0.05);
    }
    await sim.applyChassisVelocity({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 }, { maxSteps: 2000, advanceSeconds: 0 });
    await sim.advanceTime(0.4);
    sim.renderFrame();
    const servicePose = sim.getBasePose();
    const before = sim.getPresentationAlignment();

    await sim.applyArmTargets({ ...LEKIWI_ARM_POSES.pick_grip, arm_gripper: LEKIWI_COURIER_CONTROLLER.gripperOpenRad }, { maxSteps: 8000 });
    const samples = [];
    for (let i = 0; i < 24; i += 1) {
      await sim.advanceTime(0.05);
      sim.renderFrame();
      samples.push(sim.getPresentationAlignment());
    }
    await sim.applyArmTargets({ ...LEKIWI_ARM_POSES.pick_grip, arm_gripper: LEKIWI_COURIER_CONTROLLER.gripperCloseRad }, { maxSteps: 8000 });
    for (let i = 0; i < 24; i += 1) {
      await sim.advanceTime(0.05);
      sim.renderFrame();
      samples.push(sim.getPresentationAlignment());
    }
    const contacts = sim.getState().observation.contacts || [];
    const pair = (contact) => `${contact.geom1Name || ''}|${contact.geom2Name || ''}`;
    const serviceErrorM = Math.hypot(
      servicePose.xM - LEKIWI_WORKCELL.serviceStopXYM[0],
      servicePose.yM - LEKIWI_WORKCELL.serviceStopXYM[1],
    );
    const serviceYawErrorRad = Math.abs(wrap(servicePose.yawRad - LEKIWI_WORKCELL.serviceStopYawRad));
    return {
      before,
      after: samples.at(-1),
      maxError: Math.max(...samples.map((sample) => sample.maxArmPivotErrorMm || 0)),
      serviceErrorM,
      serviceYawErrorRad,
      fixedJawContact: contacts.some((contact) => pair(contact).includes('beaker') && pair(contact).includes('fixed_jaw')),
      movingJawContact: contacts.some((contact) => pair(contact).includes('beaker') && pair(contact).includes('moving_jaw')),
    };
  });
  expect(arm.serviceErrorM).toBeLessThan(0.03);
  expect(arm.serviceYawErrorRad).toBeLessThan(0.08);
  expect(arm.maxError).toBeLessThan(12);
  expect(arm.after.beakerErrorMm).toBeLessThan(1e-6);
  expect(arm.fixedJawContact).toBe(true);
  expect(arm.movingJawContact).toBe(true);
  expect(arm.after.visualWristToBeakerMm).toBeLessThan(180);
  const wristBefore = arm.before.visualPivotsMm.wrist_roll;
  const wristAfter = arm.after.visualPivotsMm.wrist_roll;
  expect(Math.hypot(...wristAfter.map((value, index) => value - wristBefore[index]))).toBeGreaterThan(40);
'''
text, count = pattern.subn(replacement, text, count=1)
if count != 1:
    raise RuntimeError(f'presentation arm test replacement expected 1 match, got {count}')
path.write_text(text)
print('LeKiwi presentation test now reaches the workcell through physical base motion before arm/grasp checks.')
