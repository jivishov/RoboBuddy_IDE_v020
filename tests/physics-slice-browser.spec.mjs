import { test, expect } from '@playwright/test';

test('Phase 1 browser MuJoCo slice uses the pinned engine and fixed simulation time', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));

  await page.goto('/physics-slice.html', { waitUntil: 'domcontentloaded' });
  await page.locator('#load').click();
  await expect(page.locator('#status')).toHaveText('Load scene complete', { timeout: 30_000 });
  await expect(page.locator('#engineVersion')).toHaveText('3.11.0');
  await expect(page.locator('#timestep')).toHaveText('0.002000 s');
  await expect(page.locator('#time')).toHaveText('0.0000 s');

  await page.locator('#right').click();
  await expect(page.locator('#status')).toHaveText('Set target +0.8 rad complete');
  await expect(page.locator('#target')).toHaveText('0.80000 rad');

  await page.locator('#step500').click();
  await expect(page.locator('#status')).toHaveText('Advance 500 steps complete', { timeout: 30_000 });
  await expect(page.locator('#time')).toHaveText('1.0000 s');
  await expect(page.locator('#contactsReadable')).toHaveText('yes');

  const boxText = await page.locator('#box').textContent();
  const boxZ = Number(String(boxText).split(',')[2]?.replace('m', '').trim());
  expect(Number.isFinite(boxZ)).toBe(true);
  expect(boxZ).toBeLessThan(1.0);
  expect(boxZ).toBeGreaterThan(0.38);

  const contactCount = Number(await page.locator('#contacts').textContent());
  expect(contactCount).toBeGreaterThan(0);

  const hingePosition = Number(String(await page.locator('#position').textContent()).replace('rad', '').trim());
  expect(Number.isFinite(hingePosition)).toBe(true);
  expect(Math.abs(hingePosition)).toBeGreaterThan(0.01);

  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
