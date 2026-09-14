import { expect, test } from '@playwright/test';
import { writeFile } from 'node:fs/promises';

const SCENE_VERSION = 'robobuddy.lab.scene.v1';
const TASK_VERSION = 'robobuddy.lab.task.v1';
const TOOL_VERSION = 'robobuddy.openarm.lab-builder.v1';

test('standalone Lab Builder authors, plans and physically evaluates a dry vial transfer', async ({ page }, testInfo) => {
  test.setTimeout(240000);
  const errors = [];
  page.on('pageerror', error => errors.push(String(error)));
  await page.goto('/tests/fixtures/openarm-lab-builder.html');
  await page.waitForFunction(() => window.ready, null, { timeout: 60000 });

  const blank = await page.evaluate(() => sim.getWorkcellState());
  expect(blank.workspaceMode).toBe('lab_builder');
  expect(blank.equipment).toHaveLength(0);
  expect(Object.keys(blank.bodies)).not.toContain('flask');
  expect(Object.keys(blank.bodies)).not.toContain('beaker');
  for (const forbidden of ['cell_table','left_hotplate','right_ring_gauze','left_source_support','right_source_support']) expect(blank.geometryIds).not.toContain(forbidden);
  expect(blank.geometryIds).toContain('mount_column');

  const names = await page.evaluate(async () => {
    const { createWebMcpRegistration } = await import('/src/webmcp/register-ide-tools.js');
    window.tools = new Map();
    Object.defineProperty(document, 'modelContext', { configurable: true, value: { registerTool: async (tool, { signal }) => { tools.set(tool.name, tool); signal.addEventListener('abort', () => tools.delete(tool.name)); } } });
    let executionState = 'idle', access = 'assist', epoch = 0;
    const app = { sim: { backend: sim, getState: () => sim.getState(), getPhysicalSession: () => sim.getPhysicalSession(), getPhysicalAuthorityToken: () => sim.getPhysicalAuthorityToken(), applyPhysicalTargets: (...args) => sim.applyPhysicalTargets(...args) }, runToken: 0,
      getExecutionState: () => executionState,
      beginExecution() { if (executionState !== 'idle') return null; executionState = 'running'; return ++this.runToken; },
      finishExecution(token) { if (token === this.runToken) executionState = 'idle'; },
      cancelExecution() { this.runToken++; executionState = 'idle'; this.openarmAgentProgramActive = false; },
      resetSimulation: () => sim.reset(), setStatus: () => {}, renderPanels: () => {} };
    const facade = { app, controlSequence: 0, activeControlId: null,
      assertActive(e) { if (e !== epoch || access !== 'assist') throw Error('Agent access inactive'); },
      setRegistrationEpoch(e) { epoch = e; },
      getRegistrationContext: () => ({ profileId: 'openarm', workspaceStatus: 'ready', simulationReady: sim.isReady(), simulationMode: 'physical_mujoco', workspaceGeneration: 1, simulatorEpoch: 1 }),
      shouldRegisterMicroduckControl: () => false };
    window.testApp = app; window.facade = facade; window.registration = createWebMcpRegistration(facade); await registration.setAccess('assist');
    window.callTool = (name, input, signal) => tools.get(name).execute(input, { signal });
    return [...tools.keys()];
  });
  for (const name of ['inspect_openarm_workcell','manage_openarm_workcell','run_openarm_program']) expect(names).toContain(name);

  const authored = await page.evaluate(async ({ SCENE_VERSION, TOOL_VERSION }) => {
    const inspection = await callTool('inspect_openarm_workcell', {});
    const scene = {
      schema_version: SCENE_VERSION, id: 'synthetic_lab_demo',
      reference: { mode: 'synthetic_fixture', label: 'software integration fixture', image_pixels_available: false, dimensions: { known_length_m: .82, description: 'declared bench width', source: 'source_provided' } },
      assumptions: ['Software integration fixture; not independent real-photo reconstruction evidence.', 'Receiver is idealized as a fixed rigid fixture; source tray is free-standing.'],
      assets: [
        { id: 'bench', kind: 'bench', position_m: [.41,0,.98], dimensions_m: [.82,1.10,.025], mass_kg: 20, dynamic: false, role: 'work surface', quantity_evidence: { source: 'source_provided' } },
        { id: 'source_tray', kind: 'tray', position_m: [.55,.1535,1.005], dimensions_m: [.10,.10,.030], mass_kg: .18, supported_by: 'bench', role: 'free-standing source tray', quantity_evidence: { source: 'source_provided' } },
        { id: 'sample', kind: 'vial', position_m: [.55,.1535,1.008], dimensions_m: [.030,.030,.130], mass_kg: .060, dynamic: true, supported_by: 'source_tray', role: 'dry transfer object', quantity_evidence: { source: 'source_provided' } },
        { id: 'receiver', kind: 'receiver', position_m: [.67,.1535,1.005], dimensions_m: [.110,.100,.050], mass_kg: .30, dynamic: false, supported_by: 'bench', role: 'accessible receiving compartment', quantity_evidence: { source: 'source_provided' } },
        { id: 'obstacle', kind: 'obstacle', position_m: [.61,-.14,1.005], dimensions_m: [.055,.055,.100], mass_kg: .20, dynamic: false, supported_by: 'bench', role: 'nearby geometry-only obstacle', quantity_evidence: { source: 'source_provided' } },
      ],
    };
    const staged = await callTool('manage_openarm_workcell', { schema_version: TOOL_VERSION, command: 'stage_scene', expected_scene_revision: inspection.authority.sceneRevision, scene_spec: scene });
    const beforeApply = sim.getPhysicalAuthorityToken();
    const applied = await callTool('manage_openarm_workcell', { schema_version: TOOL_VERSION, command: 'apply', stage_id: staged.result.id, acknowledge_reset: true });
    await sim.advanceTime(.5);
    return { inspection, staged, beforeApply, applied, state: sim.getWorkcellState() };
  }, { SCENE_VERSION, TOOL_VERSION });
  expect(authored.staged.ok).toBe(true);
  expect(authored.applied.ok, JSON.stringify(authored.applied)).toBe(true);
  expect(authored.applied.result.reset).toBe(true);
  expect(authored.state.equipment).toHaveLength(5);
  expect(authored.state.equipment.find(item => item.id === 'source_tray').dynamic).toBe(true);
  expect(authored.state.bodies.lab_source_tray).toBeTruthy();
  expect(authored.state.bodies.lab_sample).toBeTruthy();
  await page.screenshot({ path: testInfo.outputPath('authored-lab.png') });

  const planned = await page.evaluate(async ({ TASK_VERSION, TOOL_VERSION }) => {
    const task = await callTool('manage_openarm_workcell', { schema_version: TOOL_VERSION, command: 'set_task', task_spec: {
      schema_version: TASK_VERSION, id: 'transfer_sample', type: 'dry_transfer', object_id: 'sample', receiver_id: 'receiver', side: 'left', required_action_sequence: true,
      tolerances: { position_m: .015, orientation_rad: .35, settle_speed_ms: .035, settle_angular_speed_rads: .8, settle_dwell_s: .20, retreat_m: .05, max_penetration_m: .002 },
    } });
    const plan = await callTool('manage_openarm_workcell', { schema_version: TOOL_VERSION, command: 'plan_transfer' });
    return { task, plan, binding: sim.getWorkcellState().generatedPlanBinding, validation: sim.getWorkcellState().programValidation };
  }, { TASK_VERSION, TOOL_VERSION });
  expect(planned.task.ok).toBe(true);
  expect(planned.plan.ok).toBe(true);
  expect(planned.plan.result.supported, JSON.stringify(planned.plan)).toBe(true);
  expect(planned.binding.sceneRevision).toBe(authored.state.authority.sceneRevision);
  expect(planned.validation.taskSpaceClearanceValidated).toBe(true);
  expect(planned.validation.sampledRobotLinkClearanceValidated).toBe(true);
  expect(planned.validation.continuousCollisionGuarantee).toBe(false);

  const freshness = await page.evaluate(async () => {
    const plan = structuredClone(sim.programSpec);
    await sim.advanceTime(.02);
    const sample = sim.lastObservation.bodies.lab_sample;
    const stillFresh = (() => { try { sim.setLabProgramSpec(plan); return true; } catch { return false; } })();
    return { stillFresh, position: sample.positionM, binding: sim.getWorkcellState().generatedPlanBinding };
  });
  expect(freshness.stillFresh).toBe(true);

  const execution = await page.evaluate(async () => callTool('run_openarm_program', structuredClone(sim.programSpec)));
  await writeFile(testInfo.outputPath('lab-transfer-result.json'), JSON.stringify(execution, null, 2));
  expect(execution.ok, JSON.stringify(execution)).toBe(true);
  expect(execution.executionStatus).toBe('completed');
  const outcome = await page.evaluate(() => ({ evaluation: sim.getTaskEvaluation(), project: sim.exportLabProject(), state: sim.getWorkcellState(), execution: testApp.getExecutionState() }));
  expect(outcome.execution).toBe('idle');
  expect(outcome.evaluation.success, JSON.stringify(outcome.evaluation)).toBe(true);
  for (const flag of ['grasp','lift','transport','receivingRegion','support','release','settled','retreat']) expect(outcome.evaluation.flags[flag]).toBe(true);
  expect(outcome.evaluation.flags.excessivePenetration).toBe(false);
  expect(outcome.evaluation.flags.prohibitedContact).toBe(false);
  expect(outcome.project.auto_start).toBe(false);
  expect(outcome.project.scene.id).toBe('synthetic_lab_demo');
  expect(outcome.project.task.id).toBe('transfer_sample');
  expect(outcome.project.program.segments.length).toBeGreaterThan(7);
  expect(outcome.project.execution_profile.program_validation.sampledRobotLinkClearanceValidated).toBe(true);
  await page.screenshot({ path: testInfo.outputPath('completed-lab-transfer.png') });

  const estimator = await page.evaluate(async () => {
    const target = sim.lastObservation.joints.openarm_left_finger_joint1.targetRad ?? sim.lastObservation.joints.openarm_left_finger_joint1.positionRad;
    sim.setObservationProfile('synthetic_estimator_v1', { seed: 12345 });
    await sim.applyPhysicalTargets({ openarm_left_finger_joint1: target }, { maxSteps: 200, advanceSeconds: .08 });
    const first = sim.getSensorObservation();
    sim.setObservationProfile('synthetic_estimator_v1', { seed: 12345 });
    await sim.applyPhysicalTargets({ openarm_left_finger_joint1: target }, { maxSteps: 200, advanceSeconds: .08 });
    const second = sim.getSensorObservation();
    return { first, second, state: sim.getWorkcellState() };
  });
  expect(estimator.first.valid).toBe(true);
  expect(estimator.first.profileId).toBe('synthetic_estimator_v1');
  expect(estimator.first.contactsAvailable).toBe(false);
  expect(estimator.first.cameraPerception).toBe(false);
  expect(estimator.second.valid).toBe(true);
  expect(estimator.state.preHardwarePackageStatus).toContain('partial');

  expect(errors, errors.join('\n')).toEqual([]);
});