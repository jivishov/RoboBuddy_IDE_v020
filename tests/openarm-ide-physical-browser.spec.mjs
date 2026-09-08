import { expect, test } from '@playwright/test';

test('OpenArm V2 Phase 5A uses one MuJoCo authority for live Python, renderer, evaluator, and bounded WebMCP', async ({ page }) => {
  test.setTimeout(300_000);
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/?ci=openarm-phase5a', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 75_000 });
  await page.locator('#robotSelect').selectOption('openarm');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 75_000 });
  await expect(page.locator('#taskSelect')).toHaveValue('openarm-04-filtration-workcell');
  await expect(page.locator('#taskPanel')).toContainText('Bimanual Heater and Ring-Stand Stack');
  await expect(page.locator('#modeChip')).toContainText('OPENARM V2 PHYSICAL WORKSPACE');
  await expect(page.locator('#simBadge')).toContainText('ACTUAL CONTACT/GRAVITY');
  await expect(page.locator('#physicsBackendBadge')).toContainText('browser-mujoco');
  await expect(page.locator('#driverLabel')).toContainText('robobuddy.sim.v1');
  await expect(page.locator('#stepBtn')).toBeDisabled();
  await expect(page.locator('#cursorBtn')).toBeDisabled();

  const initial = await page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    return { backend: app.sim.backend?.constructor?.name, authority: app.sim.getPhysicalAuthorityToken(), source: app.files['main.py'], trajectories: app.files['trajectories.py'], prepared: app.prepared };
  });
  expect(initial.backend).toBe('OpenArmPhysicalSimulator');
  expect(initial.authority).toMatchObject({ robotId: 'openarm_v2_bimanual', sceneRevision: 'phase5a-openarm-bimanual-stack-v1' });
  expect(initial.source).toContain('from robobuddy.sim import connect');
  expect(initial.source).toContain('await robot.send_action');
  expect(initial.source).toContain('await robot.advance');
  expect(initial.source).not.toContain('time.sleep(');
  expect(initial.source).not.toContain('lerobot');
  expect(initial.trajectories).toContain('openarm_left_finger_joint1');
  expect(initial.trajectories).toContain('openarm_right_finger_joint1');
  expect(initial.prepared).toBeNull();

  await page.locator('#runBtn').click();
  await expect(page.locator('#statusMessage')).toHaveText('Run complete · OpenArm V2 physical task succeeded', { timeout: 200_000 });

  const completed = await page.evaluate(() => {
    const app = window.__robobuddyCi.app;
    const backend = app.sim.backend;
    const state = app.sim.getState();
    const observation = state.observation;
    const rendered = {};
    for (const id of ['phase5a_flask','phase5a_beaker']) rendered[id] = backend.objectMeshes.get(id).position.toArray();
    const expected = Object.fromEntries(['phase5a_flask','phase5a_beaker'].map((id) => {
      const p = observation.bodies[id].positionM;
      return [id,[p[0]*1000,p[2]*1000,-p[1]*1000]];
    }));
    const beforeRender = observation.simulationTimeSeconds;
    for (let i=0;i<12;i+=1) backend.renderFrame(i*16.7);
    const afterRender = app.sim.getState().observation.simulationTimeSeconds;
    return { evaluation: app.sim.getTaskEvaluation(), authority: app.sim.getPhysicalAuthorityToken(), rendered, expected, beforeRender, afterRender, contacts: app.sim.getContacts(), physicalRuntimeActive: app.physicalRuntime.isActive(), canvas: { authority: document.querySelector('#simCanvas').dataset.simulationAuthority, success: document.querySelector('#simCanvas').dataset.physicalTaskSuccess } };
  });
  expect(completed.evaluation.success).toBe(true);
  expect(completed.evaluation.orderViolation).toBe(false);
  for (const key of ['flask','beaker']) {
    expect(completed.evaluation[key]).toMatchObject({ contactSeen:true, liftSeen:true, carrySeen:true, releaseSeen:true, supportSeen:true, settleSeen:true, retreatSeen:true, inTarget:true, currentGripperContact:false, currentSupportContact:true });
    expect(completed.evaluation[key].maxHeldTravelM).toBeGreaterThanOrEqual(0.045);
    expect(completed.evaluation[key].settleEvidenceDurationSeconds).toBeGreaterThanOrEqual(0.20);
    expect(completed.evaluation[key].settlePositionDriftM).toBeLessThanOrEqual(0.001);
    expect(completed.evaluation[key].linearSpeedMPerS).toBeLessThanOrEqual(0.03);
  }
  expect(completed.contacts.task_success).toBe(true);
  expect(completed.canvas).toEqual({ authority:'physics-session', success:'true' });
  expect(completed.physicalRuntimeActive).toBe(false);
  expect(completed.afterRender).toBeCloseTo(completed.beforeRender, 12);
  for (const id of ['phase5a_flask','phase5a_beaker']) completed.rendered[id].forEach((value,index) => expect(value).toBeCloseTo(completed.expected[id][index], 5));

  const webmcp = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const realFacade = window.__robobuddyCi.agentFacade;
    const { executeProfileControl, WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION } = await import('/src/webmcp/robot-controls.js');
    const facade = { app, activeControlId:null, controlSequence:0, assertActive:()=>{}, getRegistrationContext:()=>app.getAgentRegistrationContext(), inspectSimulation:(snapshot)=>realFacade.inspectSimulation(snapshot) };
    await app.resetSimulation();
    const before = app.sim.getPhysicalAuthorityToken();
    const result = await executeProfileControl(facade,'openarm',{ schema_version:WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION, command:'set_joint_targets', targets_rad:{ openarm_left_joint1:0.30 }, max_steps:100, advance_seconds:0 },new AbortController().signal,1);
    let passiveError = null;
    try { await executeProfileControl(facade,'openarm',{ schema_version:WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION, command:'set_joint_targets', targets_rad:{ openarm_left_finger_joint2:0.2 } },new AbortController().signal,1); }
    catch (error) { passiveError = error.code || error.message; }
    return { before, after:app.sim.getPhysicalAuthorityToken(), result, passiveError };
  });
  expect(webmcp.result.commandStatus).toBe('accepted');
  expect(webmcp.result.acceptedTargetsRad).toEqual({ openarm_left_joint1:0.30 });
  expect(Math.abs(webmcp.result.observedState.jointsRad.openarm_left_joint1-0.30)).toBeGreaterThan(0.01);
  expect(webmcp.after.sessionId).toBe(webmcp.before.sessionId);
  expect(webmcp.after.epoch).toBe(webmcp.before.epoch);
  expect(webmcp.passiveError).toBe('INVALID_ARGUMENT');

  const stale = await page.evaluate(async () => {
    const app = window.__robobuddyCi.app;
    const { LivePythonBridge } = await import('/src/runtime/live-python-bridge.js');
    const bridge = new LivePythonBridge(app.sim.getPhysicalSession());
    await bridge.connect('openarm_v2_bimanual');
    const before = app.sim.getPhysicalAuthorityToken();
    await app.resetSimulation();
    const after = app.sim.getPhysicalAuthorityToken();
    let staleCode = null;
    try { await bridge.sendAction({ openarm_left_joint1:0.2 }, { maxSteps:10 }); } catch (error) { staleCode=error.code || null; }
    return { before, after, staleCode, evaluation:app.sim.getTaskEvaluation() };
  });
  expect(stale.after.sessionId).toBe(stale.before.sessionId);
  expect(stale.after.epoch).toBeGreaterThan(stale.before.epoch);
  expect(stale.staleCode).toBe('STALE_LIVE_SESSION');
  expect(stale.evaluation.success).toBe(false);
  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
