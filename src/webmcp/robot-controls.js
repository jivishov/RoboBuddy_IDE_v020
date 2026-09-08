import { PROFILES, validateAction } from '../profiles.js';
import { SO101_MANIPULATION_MODEL_PACKAGE } from '../physics/model-packages.js';
import { OPENARM_PHASE5A_MODEL_PACKAGE } from '../physics/openarm-model-package.js';
import { WebMcpDomainError } from './agent-facade.js';

const DIRECT_CONTROL_PROFILES = Object.freeze(['openarm', 'so101', 'lekiwi', 'unitree']);
const DIRECT_CONTROL_SET = new Set(DIRECT_CONTROL_PROFILES);
const LEKIWI_BASE_FIELDS = Object.freeze(['x.vel', 'y.vel', 'theta.vel']);
const LEKIWI_STOP_ACTION = Object.freeze({ 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 });
const MAX_LEKIWI_DURATION_MS = 3000;
const SOURCE_ACTION_APPLY_MS = 20;
const SO101_PHYSICAL_WEBMCP_VERSION = 'robobuddy.so101.physical.v1';
const OPENARM_PHYSICAL_WEBMCP_VERSION = 'robobuddy.openarm.physical.v1';

const PHYSICAL_CONFIG = Object.freeze({
  so101: Object.freeze({ modelPackage: SO101_MANIPULATION_MODEL_PACKAGE, version: SO101_PHYSICAL_WEBMCP_VERSION, maxAdvanceSeconds: 2, defaultSteps: 1000, maxSteps: 5000 }),
  openarm: Object.freeze({ modelPackage: OPENARM_PHASE5A_MODEL_PACKAGE, version: OPENARM_PHYSICAL_WEBMCP_VERSION, maxAdvanceSeconds: 2, defaultSteps: 2500, maxSteps: 10000 }),
});
const isPhysicalControlProfile = (profileId) => Boolean(PHYSICAL_CONFIG[profileId]);

function physicalRanges(modelPackage) {
  return Object.freeze(Object.fromEntries(modelPackage.actuators
    .filter((actuator) => actuator.command === 'position-rad')
    .map((actuator) => {
      const joint = modelPackage.joints.find((item) => item.id === actuator.jointId);
      if (!joint?.rangeRad || !actuator.controlRangeRad) throw new Error(`Physical WebMCP has incomplete bounds for ${actuator.jointId}`);
      const minimum = Math.max(Number(joint.rangeRad[0]), Number(actuator.controlRangeRad[0]));
      const maximum = Math.min(Number(joint.rangeRad[1]), Number(actuator.controlRangeRad[1]));
      if (!Number.isFinite(minimum) || !Number.isFinite(maximum) || minimum >= maximum) throw new Error(`Physical WebMCP has no valid control intersection for ${actuator.jointId}`);
      return [actuator.jointId, Object.freeze([minimum, maximum])];
    })));
}
const PHYSICAL_RANGES = Object.freeze(Object.fromEntries(Object.entries(PHYSICAL_CONFIG).map(([profileId, config]) => [profileId, physicalRanges(config.modelPackage)])));

const TOOL_META = Object.freeze({
  openarm: Object.freeze({
    name: 'control_openarm_simulation',
    title: 'Control OpenArm V2 physical MuJoCo simulation',
    description: `Control the active OpenArm V2 Phase 5A dry rigid-body benchmark through ${OPENARM_PHYSICAL_WEBMCP_VERSION}. Targets are explicit radians and route to the same authoritative PhysicsSession used by live Python, rendering, and task evaluation. Only source-actuated V2 joints are exposed; passive finger coupling remains inside MuJoCo. Command acceptance is not target achievement. No hardware, CAN/network, object-transform, snap, weld, qpos/qvel, save, export, or publish control is exposed.`,
  }),
  so101: Object.freeze({
    name: 'control_so101_simulation',
    title: 'Control SO-101 physical MuJoCo simulation',
    description: `Control the active SO-101 rigid-body benchmark through ${SO101_PHYSICAL_WEBMCP_VERSION}. Joint targets are explicit radians and route to the same authoritative PhysicsSession used by live Python, rendering, and task evaluation. Command acceptance is not target achievement; returned state is observed MuJoCo state. No hardware, object-transform, snap, weld, save, export, or publish control is exposed.`,
  }),
  lekiwi: Object.freeze({
    name: 'control_lekiwi_simulation', title: 'Control LeKiwi browser simulation',
    description: 'Apply one partial LeKiwi arm/base action using only configured action fields and limits, or reset the active LeKiwi source-plant simulation. Any nonzero base velocity requires a bounded duration and is automatically stopped before the tool returns. This never accesses ZMQ, physical hardware, files, networks, save, export, or publish surfaces.',
  }),
  unitree: Object.freeze({
    name: 'control_unitree_g1_simulation', title: 'Pose Unitree G1 browser simulation',
    description: 'Apply one partial pose using only the configured Unitree G1 named-joint limits, or reset to the active neutral kinematic workspace. This is browser-only pose visualization: no gait, balance, contact, Unitree SDK, network, hardware, save, export, or publish control is exposed.',
  }),
});

function invalidInput(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function assertPlainObject(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalidInput(`${label} must be an object.`); }
function assertOnlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalidInput(`Unexpected ${label} field: ${key}.`); }

function profileActionSchema(profileId) {
  const profile = PROFILES[profileId];
  return { type: 'object', properties: Object.fromEntries(Object.entries(profile.limits).map(([key, range]) => [key, { type: 'number', minimum: Number(range[0]), maximum: Number(range[1]), description: `${profile.units?.[key] || 'deg'}; configured browser-simulation envelope` }])), minProperties: 1, additionalProperties: false };
}
function physicalTargetsSchema(profileId) {
  return { type: 'object', properties: Object.fromEntries(Object.entries(PHYSICAL_RANGES[profileId]).map(([jointId, range]) => [jointId, { type: 'number', minimum: Number(range[0]), maximum: Number(range[1]), description: 'radians; physical MuJoCo actuator target' }])), minProperties: 1, additionalProperties: false };
}
function physicalControlSchema(profileId) {
  const config = PHYSICAL_CONFIG[profileId];
  return { type: 'object', oneOf: [
    { type: 'object', properties: {
      schema_version: { type: 'string', const: config.version }, command: { type: 'string', const: 'set_joint_targets' }, targets_rad: physicalTargetsSchema(profileId),
      advance_seconds: { type: 'number', minimum: 0, maximum: config.maxAdvanceSeconds, description: 'Optional bounded simulation-time advancement after latching the target; must align to the active MuJoCo timestep.' },
      max_steps: { type: 'integer', minimum: 1, maximum: config.maxSteps, description: 'Hard maximum physics-step budget for the latched command.' },
    }, required: ['schema_version', 'command', 'targets_rad'], additionalProperties: false },
    { type: 'object', properties: { schema_version: { type: 'string', const: config.version }, command: { type: 'string', const: 'reset' } }, required: ['schema_version', 'command'], additionalProperties: false },
  ] };
}

export function createProfileControlSchema(profileId) {
  if (!DIRECT_CONTROL_SET.has(profileId)) throw new Error(`Unsupported direct-control profile: ${profileId}`);
  if (isPhysicalControlProfile(profileId)) return physicalControlSchema(profileId);
  const setActionProperties = { command: { type: 'string', const: 'set_action' }, action: profileActionSchema(profileId) };
  if (profileId === 'lekiwi') setActionProperties.duration_ms = { type: 'integer', minimum: 20, maximum: MAX_LEKIWI_DURATION_MS, description: 'Required for any nonzero x.vel, y.vel, or theta.vel. The base is stopped automatically after this modeled duration.' };
  return { type: 'object', oneOf: [
    { type: 'object', properties: setActionProperties, required: ['command', 'action'], additionalProperties: false },
    { type: 'object', properties: { command: { type: 'string', const: 'reset' } }, required: ['command'], additionalProperties: false },
  ] };
}

export function getProfileControlDefinition(facade) {
  const context = facade.getRegistrationContext(); const profileId = context.profileId;
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || !DIRECT_CONTROL_SET.has(profileId)) return null;
  if (isPhysicalControlProfile(profileId) && context.simulationMode !== 'physical_mujoco') return null;
  return { profileId, ...TOOL_META[profileId], inputSchema: createProfileControlSchema(profileId) };
}

function parsePhysicalInput(profileId, input) {
  const config = PHYSICAL_CONFIG[profileId]; const ranges = PHYSICAL_RANGES[profileId];
  assertPlainObject(input);
  if (input.schema_version !== config.version) invalidInput(`schema_version must be ${config.version}.`);
  if (input.command === 'reset') { assertOnlyKeys(input, ['schema_version', 'command']); return { command: 'reset', schemaVersion: config.version }; }
  if (input.command !== 'set_joint_targets') invalidInput(`physical ${PROFILES[profileId].shortLabel} command must be set_joint_targets or reset.`);
  assertOnlyKeys(input, ['schema_version', 'command', 'targets_rad', 'advance_seconds', 'max_steps']);
  assertPlainObject(input.targets_rad, 'targets_rad');
  const entries = Object.entries(input.targets_rad); if (!entries.length) invalidInput('targets_rad must contain at least one physical joint target.');
  const targetsRad = {};
  for (const [jointId, raw] of entries) {
    const range = ranges[jointId]; if (!range) invalidInput(`Unknown or passive physical ${PROFILES[profileId].shortLabel} joint: ${jointId}.`);
    const value = Number(raw); if (!Number.isFinite(value)) invalidInput(`${jointId} must be a finite radian value.`);
    if (value < range[0] || value > range[1]) invalidInput(`${jointId}=${value} rad is outside ${range[0]}..${range[1]} rad.`);
    targetsRad[jointId] = value;
  }
  const advanceSeconds = input.advance_seconds == null ? 0 : Number(input.advance_seconds);
  if (!Number.isFinite(advanceSeconds) || advanceSeconds < 0 || advanceSeconds > config.maxAdvanceSeconds) invalidInput(`advance_seconds must be between 0 and ${config.maxAdvanceSeconds}.`);
  const maxSteps = input.max_steps == null ? config.defaultSteps : input.max_steps;
  if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > config.maxSteps) invalidInput(`max_steps must be an integer from 1 to ${config.maxSteps}.`);
  return { command: 'set_joint_targets', schemaVersion: config.version, targetsRad, advanceSeconds, maxSteps };
}

function parseInput(profileId, input) {
  if (isPhysicalControlProfile(profileId)) return parsePhysicalInput(profileId, input);
  assertPlainObject(input);
  if (input.command === 'reset') { assertOnlyKeys(input, ['command']); return { command: 'reset' }; }
  if (input.command !== 'set_action') invalidInput('command must be set_action or reset.');
  const topLevelKeys = profileId === 'lekiwi' ? ['command', 'action', 'duration_ms'] : ['command', 'action']; assertOnlyKeys(input, topLevelKeys); assertPlainObject(input.action, 'action');
  let action; try { action = validateAction(profileId, input.action); } catch (error) { invalidInput(error.message); }
  if (profileId !== 'lekiwi') return { command: 'set_action', action };
  const movingBase = LEKIWI_BASE_FIELDS.some((key) => Math.abs(Number(action[key] ?? 0)) > 1e-12);
  if (movingBase) { if (!Number.isInteger(input.duration_ms) || input.duration_ms < 20 || input.duration_ms > MAX_LEKIWI_DURATION_MS) invalidInput(`Nonzero LeKiwi base velocity requires duration_ms between 20 and ${MAX_LEKIWI_DURATION_MS}.`); }
  else if (input.duration_ms !== undefined) invalidInput('duration_ms is only accepted when the LeKiwi action includes a nonzero base velocity.');
  return { command: 'set_action', action, durationMs: movingBase ? input.duration_ms : null, movingBase };
}

function captureControlContext(facade, profileId, expectedEpoch) {
  facade.assertActive(expectedEpoch); const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready') throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The RoboBuddy workspace is not ready.', { retryable: true });
  if (context.profileId !== profileId || !DIRECT_CONTROL_SET.has(context.profileId)) throw new WebMcpDomainError('PROFILE_MISMATCH', `This tool requires the active ${PROFILES[profileId].shortLabel} workspace.`);
  if (!context.simulationReady) throw new WebMcpDomainError('SIMULATION_NOT_READY', `${PROFILES[profileId].shortLabel} browser simulation is not ready.`, { retryable: true });
  if (isPhysicalControlProfile(profileId) && context.simulationMode !== 'physical_mujoco') throw new WebMcpDomainError('PROFILE_MISMATCH', `${PROFILES[profileId].shortLabel} direct control requires the physical MuJoCo workspace.`);
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Stop or finish the active Python run before direct WebMCP simulation control.', { retryable: true });
  const physicalAuthority = isPhysicalControlProfile(profileId) ? facade.app.sim.getPhysicalAuthorityToken?.() : null;
  if (isPhysicalControlProfile(profileId) && !physicalAuthority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', `${PROFILES[profileId].shortLabel} physical authority is unavailable.`, { retryable: true });
  return Object.freeze({ profileId, workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, physicalAuthority: physicalAuthority ? Object.freeze({ ...physicalAuthority }) : null });
}
function samePhysicalAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }
function assertControlCurrent(facade, baseline, expectedEpoch, signal, { allowPhysicalAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The WebMCP robot-control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch); const current = facade.getRegistrationContext();
  if (current.workspaceStatus !== 'ready' || !current.simulationReady || current.profileId !== baseline.profileId || current.workspaceGeneration !== baseline.workspaceGeneration || current.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active robot, workspace, or simulator backend changed during the WebMCP control call.', { retryable: true });
  if (isPhysicalControlProfile(baseline.profileId) && !allowPhysicalAuthorityChange) {
    const authority = facade.app.sim.getPhysicalAuthorityToken?.(); if (!samePhysicalAuthority(baseline.physicalAuthority, authority)) throw new WebMcpDomainError('OPERATION_CANCELLED', `The ${PROFILES[baseline.profileId].shortLabel} PhysicsSession epoch changed during this WebMCP call.`, { retryable: true });
  }
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during WebMCP control.', { retryable: true });
  return current;
}
function canSafelyStopLeKiwi(facade, baseline, expectedEpoch) { try { facade.assertActive(expectedEpoch); const current = facade.getRegistrationContext(); return current.workspaceStatus === 'ready' && current.simulationReady && current.profileId === 'lekiwi' && current.workspaceGeneration === baseline.workspaceGeneration && current.simulatorEpoch === baseline.simulatorEpoch && facade.app.getExecutionState() === 'idle'; } catch { return false; } }
async function autoStopLeKiwi(facade, baseline, expectedEpoch) { if (!canSafelyStopLeKiwi(facade, baseline, expectedEpoch)) return false; try { return (await facade.app.sim.applyAction(LEKIWI_STOP_ACTION)) !== false; } catch { return false; } }
function simulationResult(facade, profileId, command, extras = {}) { const snapshot = facade.app.getAgentSnapshot(); return { ok: true, profileId, robot: PROFILES[profileId].label, command, hardwareValidated: false, ...extras, simulation: snapshot.workspaceStatus === 'ready' ? facade.inspectSimulation(snapshot) : { workspaceStatus: snapshot.workspaceStatus } }; }
function observedPhysicalState(observation) { return { simulationTimeSeconds: Number(observation?.simulationTimeSeconds ?? 0), jointsRad: Object.fromEntries(Object.entries(observation?.joints || {}).map(([jointId, state]) => [jointId, Number(state.positionRad)])), bodies: structuredClone(observation?.bodies || {}), contactCount: Number(observation?.contactCount || 0) }; }

export async function executeProfileControl(facade, profileId, input, signal, expectedEpoch) {
  if (!DIRECT_CONTROL_SET.has(profileId)) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This direct-control tool is not available for the requested profile.');
  const parsed = parseInput(profileId, input); const baseline = captureControlContext(facade, profileId, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded WebMCP robot-control call is still active.', { retryable: true });
  const controlId = `webmcp-${profileId}-${expectedEpoch}-${++facade.controlSequence}`; facade.activeControlId = controlId;
  const beforeTick = () => { assertControlCurrent(facade, baseline, expectedEpoch, signal); return true; };
  let lekiwiBaseStarted = false;
  try {
    assertControlCurrent(facade, baseline, expectedEpoch, signal);
    if (parsed.command === 'reset') {
      const reset = await facade.app.resetSimulation(); if (!reset) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'The browser simulation could not be reset.', { retryable: true });
      assertControlCurrent(facade, baseline, expectedEpoch, signal, { allowPhysicalAuthorityChange: isPhysicalControlProfile(profileId) });
      facade.app.setStatus?.(`Agent reset ${PROFILES[profileId].shortLabel} browser simulation`); facade.app.renderPanels?.();
      return simulationResult(facade, profileId, 'reset', { reset: true, schemaVersion: isPhysicalControlProfile(profileId) ? PHYSICAL_CONFIG[profileId].version : undefined, physicalAuthority: isPhysicalControlProfile(profileId) ? facade.app.sim.getPhysicalAuthorityToken?.() : undefined });
    }
    if (isPhysicalControlProfile(profileId)) {
      const result = await facade.app.sim.applyPhysicalTargets(parsed.targetsRad, { maxSteps: parsed.maxSteps, advanceSeconds: parsed.advanceSeconds });
      assertControlCurrent(facade, baseline, expectedEpoch, signal); facade.app.setStatus?.(`Agent applied bounded ${PROFILES[profileId].shortLabel} physical MuJoCo target`); facade.app.renderPanels?.();
      return simulationResult(facade, profileId, parsed.command, { schemaVersion: PHYSICAL_CONFIG[profileId].version, commandStatus: result.status, commandId: result.commandId, acceptedTargetsRad: result.acceptedTargetsRad, advanceSeconds: parsed.advanceSeconds, observedState: observedPhysicalState(result.observation), taskEvaluation: result.taskEvaluation, physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() });
    }
    const applied = await facade.app.sim.applyAction(parsed.action, { beforeTick }); if (applied === false) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The browser simulation stopped before the action completed.', { retryable: true });
    lekiwiBaseStarted = profileId === 'lekiwi' && parsed.movingBase;
    if (lekiwiBaseStarted) {
      const remainingSeconds = Math.max(0, parsed.durationMs - SOURCE_ACTION_APPLY_MS) / 1000;
      if (remainingSeconds > 0 && (await facade.app.sim.advanceTime(remainingSeconds, { realtime: true, beforeTick })) === false) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The LeKiwi modeled base motion stopped before its bounded duration completed.', { retryable: true });
      await autoStopLeKiwi(facade, baseline, expectedEpoch); lekiwiBaseStarted = false;
    }
    assertControlCurrent(facade, baseline, expectedEpoch, signal); facade.app.setStatus?.(`Agent applied bounded ${PROFILES[profileId].shortLabel} browser-simulation action`); facade.app.renderPanels?.();
    return simulationResult(facade, profileId, 'set_action', { appliedAction: parsed.action, durationMs: parsed.durationMs, baseAutoStopped: profileId === 'lekiwi' ? Boolean(parsed.movingBase) : undefined });
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || 'The browser simulation rejected the bounded robot action.').slice(0, 280), { retryable: true, details: { profileId } });
  } finally {
    if (lekiwiBaseStarted) await autoStopLeKiwi(facade, baseline, expectedEpoch);
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}

export const WEBMCP_DIRECT_CONTROL_PROFILES = DIRECT_CONTROL_PROFILES;
export const WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION = SO101_PHYSICAL_WEBMCP_VERSION;
export const WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION = OPENARM_PHYSICAL_WEBMCP_VERSION;
