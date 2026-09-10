import { expect, test } from '@playwright/test';

async function openPhysical(page, task = 'microduck-physical-locomotion') {
  await page.addInitScript(() => {
    const registrations = [];
    Object.defineProperty(document, 'modelContext', { configurable: true, value: {
      registerTool(tool, options = {}) { registrations.push({ tool, signal: options.signal }); return Promise.resolve(); },
    } });
    window.__interfaceRegistrations = registrations;
  });
  await page.goto('/?ci=microduck-interfaces', { waitUntil: 'domcontentloaded' });
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 60_000 });
  await page.locator('#robotSelect').selectOption('microduck');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await page.locator('#taskSelect').selectOption(task);
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  await expect(page.locator('#simCanvas')).toHaveAttribute('data-microduck-workspace', 'physical');
}
async function invoke(page, input) {
  return page.evaluate(async (request) => {
    const entry = window.__interfaceRegistrations.findLast(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted);
    if (!entry) throw new Error('The physical MicroDuck WebMCP tool was not registered');
    return entry.tool.execute({ schema_version: 'robobuddy.microduck.physical.v1', ...request }, { signal: new AbortController().signal });
  }, input);
}

test('registered MicroDuck WebMCP commands reach the real controller through the IDE', async ({ page }) => {
  test.setTimeout(240_000);
  const errors = [];
  page.on('pageerror', (error) => errors.push(String(error)));
  await openPhysical(page);
  await page.locator('[data-agent-access="assist"]').click();
  await expect.poll(() => page.evaluate(() => window.__interfaceRegistrations.some(({ tool, signal }) => tool.name === 'control_microduck_physical_simulation' && !signal?.aborted))).toBe(true);
  const move = await invoke(page, { command: 'set_command', request: { vx: 0.35 }, advance_seconds: 0.1 });
  expect(move.ok, JSON.stringify(move)).toBe(true);
  expect(move.controller.policyId).toBe('walking');
  expect(move.actual.simulationTimeSeconds).toBeCloseTo(0.1, 8);
  const zero = await invoke(page, { command: 'advance', advance_seconds: 0 });
  expect(zero.ok, JSON.stringify(zero)).toBe(true);
  expect(zero.executedTicks).toBe(0);
  expect(zero.actual.simulationTimeSeconds).toBe(move.actual.simulationTimeSeconds);
  const bad = await invoke(page, { command: 'set_command', request: { vx: null } });
  expect(bad).toMatchObject({ ok: false, error: { code: 'INVALID_ARGUMENT' } });
  const mismatch = await invoke(page, { command: 'request_skill', skill: 'kick_right' });
  expect(mismatch).toMatchObject({ ok: false, error: { code: 'PLANT_MISMATCH' } });
  await page.locator('#taskSelect').selectOption('microduck-physical-groundcontact');
  await expect(page.locator('#statusMessage')).toContainText('Ready', { timeout: 120_000 });
  const sit = await invoke(page, { command: 'request_skill', skill: 'sit', advance_seconds: 0.1 });
  expect(sit.ok, JSON.stringify(sit)).toBe(true);
  expect(sit.controller.policyId).toBe('sitstand');
  const rise = await invoke(page, { command: 'request_skill', skill: 'stand_up', advance_seconds: 0.1 });
  expect(rise.ok, JSON.stringify(rise)).toBe(true);
  expect(rise.controller.policyId).toBe('sitstand');
  const roller = await invoke(page, { command: 'request_skill', skill: 'roller' });
  expect(roller).toMatchObject({ ok: false, error: { code: 'CAPABILITY_UNSUPPORTED' } });
  expect(errors).toEqual([]);
});

for (const task of ['microduck-physical-locomotion', 'microduck-physical-groundcontact', 'microduck-physical-kick']) {
  test(`${task} default Python starter executes through the real IDE bridge`, async ({ page }) => {
    test.setTimeout(240_000);
    const errors = [];
    page.on('pageerror', (error) => errors.push(String(error)));
    await openPhysical(page, task);
    const result = await page.evaluate(async () => {
      const app = window.__robobuddyCi.app;
      const source = app.editor.cm.getValue();
      await app.run();
      return { source, console: app.console, problems: app.problems, state: app.sim.getState(), execution: app.getExecutionState() };
    });
    expect(result.source).toContain('MicroDuck physical MuJoCo workspace');
    expect(result.source).not.toContain('benchmark_block');
    expect(result.console.stderr, JSON.stringify(result.problems)).toBe('');
    expect(result.console.stdout, JSON.stringify(result.problems)).toContain('MicroDuck physical observation');
    expect(result.console.stdout).toContain('Measured task report');
    expect(result.execution).toBe('idle');
    expect(result.state.actual.simulationTimeSeconds).toBeGreaterThan(0);
    expect(result.state.hardwareValidated).toBe(false);
    expect(errors).toEqual([]);
  });
}
