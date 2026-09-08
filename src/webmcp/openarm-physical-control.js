import { PROFILES } from '../profiles.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from '../physics/openarm-model-package.js';
import { WebMcpDomainError } from './agent-facade.js';

export const WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION = 'robobuddy.openarm.physical.v1';
const MAX_ADVANCE_SECONDS = 2;
const DEFAULT_COMMAND_STEPS = 2000;
const MAX_COMMAND_STEPS = 5000;

const RANGES = Object.freeze(Object.fromEntries(
  OPENARM_V2_PHASE5A_MODEL_PACKAGE.joints.map((joint) => {
    const actuator = OPENARM_V2_PHASE5A_MODEL_PACKAGE.actuators.find((item) => item.jointId === joint.id);
    if (!actuator) return [joint.id, null];
    return [joint.id, Object.freeze([
      Math.max(Number(joint.rangeRad[0]), Number(actuator.controlRangeRad[0])),
      Math.min(Number(joint.rangeRad[1]), Number(actuator.controlRangeRad[1])),
    ])];
  }).filter(([, range]) => range),
));

function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`); }
function onlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected ${label} field: ${key}.`); }

function targetsSchema() {
  return {
    type: 'object',
    properties: Object.fromEntries(Object.entries(RANGES).map(([jointId, range]) => [jointId, { type: 'number', minimum: range[0], maximum: range[1], description: 'radians; OpenArm V2 MuJoCo position target' }])),
    minProperties: 1,
    additionalProperties: false,
  };
}
export function createOpenArmPhysicalControlSchema() {
  return {
    type: 'object',
    oneOf: [
      {
        type: 'object',
        properties: {
          schema_version: { type: 'string', const: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION },
          command: { type: 'string', const: 'set_joint_targets' },
          targets_rad: targetsSchema(),
          advance_seconds: { type: 'number', minimum: 0, maximum: MAX_ADVANCE_SECONDS, description: 'Optional bounded simulation-time advancement; must align to the 0.001 s V2 timestep.' },
          max_steps: { type: 'integer', minimum: 1, maximum: MAX_COMMAND_STEPS, description: 'Hard physics-step budget for the latched command.' },
        },
        required: ['schema_version', 'command', 'targets_rad'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: { schema_version: { type: 'string', const: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION }, command: { type: 'string', const: 'reset' } },
        required: ['schema_version', 'command'],
        additionalProperties: false,
      },
    ],
    additionalProperties: false,
  };
}

export function getOpenArmPhysicalControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'control_openarm_simulation',
    title: 'Control OpenArm V2 physical MuJoCo simulation',
    description: `Control the active OpenArm V2 bimanual dry-stack workspace through ${WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION}. Targets are explicit radians and route to the same PhysicsSession used by live Python, rendering and task evaluation. No qpos/qvel writes, object transforms, grasp welds, snap/attach operations, hardware transport, save, export or publish capability is exposed.`,
    inputSchema: createOpenArmPhysicalControlSchema(),
  };
}

function parse(input) {
  plain(input);
  if (input.schema_version !== WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION) invalid(`schema_version must be ${WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION}.`);
  if (input.command === 'reset') {
    onlyKeys(input, ['schema_version', 'command']);
    return { command: 'reset' };
  }
  if (input.command !== 'set_joint_targets') invalid('OpenArm physical command must be set_joint_targets or reset.');
  onlyKeys(input, ['schema_version', 'command', 'targets_rad', 'advance_seconds', 'max_steps']);
  plain(input.targets_rad, 'targets_rad');
  const entries = Object.entries(input.targets_rad);
  if (!entries.length) invalid('targets_rad must contain at least one joint target.');
  const targetsRad = {};
  for (const [jointId, raw] of entries) {
    const range = RANGES[jointId];
    if (!range) invalid(`Unknown OpenArm V2 physical joint: ${jointId}.`);
    const value = Number(raw);
    if (!Number.isFinite(value) || value < range[0] || value > range[1]) invalid(`${jointId} must be within ${range[0]}..${range[1]} radians.`);
    targetsRad[jointId] = value;
  }
  const advanceSeconds = input.advance_seconds == null ? 0 : Number(input.advance_seconds);
  if (!Number.isFinite(advanceSeconds) || advanceSeconds < 0 || advanceSeconds > MAX_ADVANCE_SECONDS) invalid(`advance_seconds must be between 0 and ${MAX_ADVANCE_SECONDS}.`);
  const maxSteps = input.max_steps == null ? DEFAULT_COMMAND_STEPS : input.max_steps;
  if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > MAX_COMMAND_STEPS) invalid(`max_steps must be an integer from 1 to ${MAX_COMMAND_STEPS}.`);
  return { command: 'set_joint_targets', targetsRad, advanceSeconds, maxSteps };
}
function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }
function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready OpenArm V2 physical workspace.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before direct OpenArm control.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'OpenArm PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }) });
}
function assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The OpenArm control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco' || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active OpenArm workspace or simulator changed during control.', { retryable: true });
  if (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The OpenArm PhysicsSession epoch changed during control.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during direct control.', { retryable: true });
}
function observed(observation) {
  return {
    simulationTimeSeconds: Number(observation?.simulationTimeSeconds || 0),
    jointsRad: Object.fromEntries(Object.entries(observation?.joints || {}).map(([id, state]) => [id, Number(state.positionRad)])),
    flaskPositionM: observation?.bodies?.flask?.positionM ? [...observation.bodies.flask.positionM] : null,
    beakerPositionM: observation?.bodies?.beaker?.positionM ? [...observation.bodies.beaker.positionM] : null,
    contactCount: Number(observation?.contactCount || 0),
  };
}

export async function executeOpenArmPhysicalControl(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-openarm-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  try {
    assertCurrent(facade, baseline, expectedEpoch, signal);
    if (parsed.command === 'reset') {
      if (!(await facade.app.resetSimulation())) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'OpenArm physical simulation could not be reset.', { retryable: true });
      assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange: true });
      return { ok: true, profileId: 'openarm', robot: PROFILES.openarm.label, command: 'reset', schemaVersion: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION, hardwareValidated: false, reset: true, physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }
    const result = await facade.app.sim.applyPhysicalTargets(parsed.targetsRad, { maxSteps: parsed.maxSteps, advanceSeconds: parsed.advanceSeconds });
    assertCurrent(facade, baseline, expectedEpoch, signal);
    facade.app.setStatus?.('Agent applied bounded OpenArm V2 physical MuJoCo target');
    facade.app.renderPanels?.();
    return { ok: true, profileId: 'openarm', robot: PROFILES.openarm.label, command: 'set_joint_targets', schemaVersion: WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION, hardwareValidated: false, commandStatus: result.status, commandId: result.commandId, acceptedTargetsRad: result.acceptedTargetsRad, advanceSeconds: parsed.advanceSeconds, observedState: observed(result.observation), taskEvaluation: result.taskEvaluation, physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 280), { retryable: true, details: { profileId: 'openarm' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}