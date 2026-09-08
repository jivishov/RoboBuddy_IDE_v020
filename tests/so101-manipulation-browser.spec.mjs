import { existsSync, readFileSync } from 'node:fs';
import { test, expect } from '@playwright/test';

const NATIVE_REFERENCE_PATH = '/tmp/robobuddy-so101-transfer-positive.json';
const SUPPORT_Z = 0.227;
const BLOCK_HALF_Z = 0.007;
const TARGET_CENTER = [0.358, -0.156];
const TARGET_HALF = [0.020, 0.030];
const CONTACT_RICH_PARITY_M = 0.02;
const OBSERVATION_SAMPLE_STEPS = 20;

function distance3(a, b) {
  return Math.sqrt(a.reduce((sum, value, index) => sum + ((Number(value) - Number(b[index])) ** 2), 0));
}

test('SO-101 P4 performs a physical browser block transfer through contact, carry, release, and rest', async ({ page }) => {
  const pageErrors = [];
  page.on('pageerror', (error) => pageErrors.push(String(error?.stack || error)));
  await page.goto('/physics-slice.html', { waitUntil: 'domcontentloaded' });

  const result = await page.evaluate(async ({ supportZ, blockHalfZ, targetCenter, targetHalf, observationSampleSteps }) => {
    const [{ PhysicsSession }, { BrowserMuJoCoBackend }, { SO101_MANIPULATION_SCENE, SO101_BENCHMARK_TRANSFER_CONTROLLER }] = await Promise.all([
      import('/src/physics/session.js'),
      import('/src/physics/browser-mujoco-backend.js'),
      import('/src/physics/so101-scene.js'),
    ]);
    const backend = new BrowserMuJoCoBackend();
    const session = new PhysicsSession(backend, { sessionId: 'so101-p4-browser' });
    const blockGripperContact = (observation) => (observation.contacts || []).some((contact) => {
      const a = contact.geom1Name;
      const b = contact.geom2Name;
      return (a === 'benchmark_block_geom' && /^(fixed_jaw_|moving_jaw_)/.test(b || ''))
        || (b === 'benchmark_block_geom' && /^(fixed_jaw_|moving_jaw_)/.test(a || ''));
    });
    const blockPosition = (observation) => [...observation.bodies.benchmark_block.positionM];

    try {
      const loaded = await session.loadScene(structuredClone(SO101_MANIPULATION_SCENE));
      const diagnostics = await session.getDiagnostics();
      const initialPositionM = blockPosition(loaded.observation);
      let observation = loaded.observation;
      let maxBlockZM = initialPositionM[2];
      let gripperContactSamples = 0;
      let carriedContactSamples = 0;
      const stageEnds = [];
      const settleSamples = [];
      let commandIndex = 0;
      const physicsDt = loaded.observation.engine.timestepSeconds;
      const controllerPeriod = SO101_BENCHMARK_TRANSFER_CONTROLLER.controllerPeriodSeconds;
      const stepsPerControllerPeriod = Math.round(controllerPeriod / physicsDt);
      if (Math.abs((stepsPerControllerPeriod * physicsDt) - controllerPeriod) > 1e-12) throw new Error('P4 controller period does not align with physics timestep');
      if (observationSampleSteps % stepsPerControllerPeriod !== 0) throw new Error('P4 observation sampling must be an integer multiple of the controller interval');

      const stages = SO101_BENCHMARK_TRANSFER_CONTROLLER.stages;
      for (let stageIndex = 0; stageIndex < stages.length; stageIndex += 1) {
        const stage = stages[stageIndex];
        const stageSteps = Math.round(stage.durationSeconds / physicsDt);
        if (Math.abs((stageSteps * physicsDt) - stage.durationSeconds) > 1e-12) throw new Error(`P4 stage ${stage.name} does not align with physics timestep`);
        const controllerIntervals = Math.round(stage.durationSeconds / controllerPeriod);
        if (Math.abs((controllerIntervals * controllerPeriod) - stage.durationSeconds) > 1e-12) throw new Error(`P4 stage ${stage.name} does not align with controller period`);

        if (Object.keys(stage.targetsRad).length) {
          commandIndex += 1;
          // Keep one bounded command alive through any following no-command settle stages.
          // This changes only the command budget; the stage-constant target and exact physics steps are unchanged.
          let commandBudgetSteps = stageSteps;
          for (let futureIndex = stageIndex + 1; futureIndex < stages.length; futureIndex += 1) {
            const future = stages[futureIndex];
            if (Object.keys(future.targetsRad).length) break;
            commandBudgetSteps += Math.round(future.durationSeconds / physicsDt);
          }
          await session.sendCommand(
            { type: 'set_joint_targets', targetsRad: { ...stage.targetsRad } },
            { commandId: `p4-stage-${commandIndex}-${stage.name}`, maxSteps: commandBudgetSteps },
          );
        }

        for (let advancedSteps = 0; advancedSteps < stageSteps;) {
          const chunkSteps = Math.min(observationSampleSteps, stageSteps - advancedSteps);
          observation = await session.advanceSteps(chunkSteps);
          advancedSteps += chunkSteps;
          const position = blockPosition(observation);
          maxBlockZM = Math.max(maxBlockZM, position[2]);
          const contacting = blockGripperContact(observation);
          if (contacting) {
            gripperContactSamples += 1;
            if (position[2] > supportZ + blockHalfZ + 0.020) carriedContactSamples += 1;
          }
          if (stage.name === 'settle_final') settleSamples.push(position);
        }
        stageEnds.push({
          name: stage.name,
          positionM: blockPosition(observation),
          gripperContact: blockGripperContact(observation),
        });
      }

      const finalPositionM = blockPosition(observation);
      const horizontalTravelM = Math.hypot(finalPositionM[0] - initialPositionM[0], finalPositionM[1] - initialPositionM[1]);
      const lifted = maxBlockZM > supportZ + blockHalfZ + 0.030;
      const physicallyCarried = lifted && carriedContactSamples >= 4 && horizontalTravelM > 0.05;
      const inTarget = Math.abs(finalPositionM[0] - targetCenter[0]) <= targetHalf[0]
        && Math.abs(finalPositionM[1] - targetCenter[1]) <= targetHalf[1];
      const released = !blockGripperContact(observation);
      const settleTail = settleSamples.slice(-3);
      const settleMotionM = settleTail.length >= 2
        ? Math.sqrt(settleTail[0].reduce((sum, value, index) => sum + ((value - settleTail.at(-1)[index]) ** 2), 0))
        : Number.POSITIVE_INFINITY;
      const resting = Math.abs(finalPositionM[2] - (supportZ + blockHalfZ)) < 0.012 && settleMotionM < 1e-5;
      return {
        model: loaded.observation.model,
        engine: loaded.observation.engine,
        diagnostics,
        simulationTimeSeconds: observation.simulationTimeSeconds,
        initialPositionM,
        finalPositionM,
        maxBlockZM,
        horizontalTravelM,
        gripperContactSamples,
        carriedContactSamples,
        lifted,
        physicallyCarried,
        inTarget,
        released,
        resting,
        settleMotionM,
        stageEnds,
        observationSampleSteps,
        stepsPerControllerPeriod,
      };
    } finally {
      session.dispose();
    }
  }, {
    supportZ: SUPPORT_Z,
    blockHalfZ: BLOCK_HALF_Z,
    targetCenter: TARGET_CENTER,
    targetHalf: TARGET_HALF,
    observationSampleSteps: OBSERVATION_SAMPLE_STEPS,
  });

  expect(result.model.id).toBe('robobuddy-so101-manipulation-v1');
  expect(result.model.asset).toBe('models/so101/manipulation.xml');
  expect(result.model.sha256).toMatch(/^[0-9a-f]{64}$/);
  expect(result.engine.version).toBe('3.11.0');
  expect(result.engine.timestepSeconds).toBeCloseTo(0.005, 12);
  expect(result.diagnostics.modelPackageId).toBe('so101-manipulation-menagerie-8161bba-v1');
  expect(result.stepsPerControllerPeriod).toBe(4);
  expect(result.observationSampleSteps).toBe(20);
  expect(result.simulationTimeSeconds).toBeCloseTo(5.1, 9);
  expect(result.gripperContactSamples).toBeGreaterThanOrEqual(4);
  expect(result.carriedContactSamples).toBeGreaterThanOrEqual(4);
  expect(result.lifted).toBe(true);
  expect(result.physicallyCarried).toBe(true);
  expect(result.horizontalTravelM).toBeGreaterThan(0.05);
  expect(result.inTarget).toBe(true);
  expect(result.released).toBe(true);
  expect(result.resting).toBe(true);
  expect(result.settleMotionM).toBeLessThan(1e-5);
  expect(result.stageEnds.find(({ name }) => name === 'close')?.gripperContact).toBe(true);
  expect(result.stageEnds.find(({ name }) => name === 'lift')?.gripperContact).toBe(true);
  expect(result.stageEnds.find(({ name }) => name === 'move')?.gripperContact).toBe(true);
  expect(result.stageEnds.find(({ name }) => name === 'settle_final')?.gripperContact).toBe(false);

  if (existsSync(NATIVE_REFERENCE_PATH)) {
    const native = JSON.parse(readFileSync(NATIVE_REFERENCE_PATH, 'utf8'));
    expect(native.metrics.success).toBe(true);
    expect(native.metrics.physicallyCarried).toBe(true);
    expect(native.metrics.released).toBe(true);
    expect(native.metrics.resting).toBe(true);
    expect(distance3(result.finalPositionM, native.metrics.finalPositionM)).toBeLessThanOrEqual(CONTACT_RICH_PARITY_M);
    expect(Math.abs(result.maxBlockZM - native.metrics.maxBlockZM)).toBeLessThanOrEqual(CONTACT_RICH_PARITY_M);
    expect(Math.abs(result.horizontalTravelM - native.metrics.horizontalTravelM)).toBeLessThanOrEqual(CONTACT_RICH_PARITY_M);
  }

  expect(pageErrors, pageErrors.join('\n\n')).toEqual([]);
});
