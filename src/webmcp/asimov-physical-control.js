import { PROFILES } from '../profiles.js';
import { ASIMOV_SOURCE } from '../physics/asimov-generated.js';
import { ASIMOV_LIMITATIONS } from '../physics/asimov-model-package.js';
import { WebMcpDomainError } from './agent-facade.js';

// Opt-in bounded access to the same Asimov PhysicsSession used by Python and rendering.
export const WEBMCP_ASIMOV_SCHEMA_VERSION = 'robobuddy.asimov.physical.v1';
const MAX_ADVANCE_SECONDS = 2;
const DEFAULT_COMMAND_STEPS = 4000;
const MAX_COMMAND_STEPS = 40000;
const JOINT_RANGES=Object.freeze(Object.fromEntries(ASIMOV_SOURCE.joints.map(j=>[j.id,j.rangeRad])));

function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`); }
function onlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected ${label} field: ${key}.`); }

export function createAsimovControlSchema() {
  const version = { type: 'string', const: WEBMCP_ASIMOV_SCHEMA_VERSION };
  const targets = {
    type: 'object',
    properties: Object.fromEntries(Object.entries(JOINT_RANGES).map(([jointId, range]) => [jointId, {
      type: 'number', minimum: range[0], maximum: range[1],
      description: 'radians; a bounded low-level position target, not an achieved position',
    }])),
    minProperties: 1,
    additionalProperties: false,
  };
  const bare = (command) => ({
    type: 'object',
    properties: { schema_version: version, command: { type: 'string', const: command } },
    required: ['schema_version', 'command'],
    additionalProperties: false,
  });
  return {
    type: 'object',
    oneOf: [
      bare('inspect_capability'),
      bare('read_state'),
      bare('read_sensors'),
      bare('engage_stand'),
      bare('stop'),
      bare('reset'),
      {
        type: 'object',
        properties: {
          schema_version: version,
          command: { type: 'string', const: 'set_joint_targets' },
          targets_rad: targets,
          advance_seconds: { type: 'number', minimum: 0, maximum: MAX_ADVANCE_SECONDS, description: 'Optional bounded simulation-time advancement.' },
          max_steps: { type: 'integer', minimum: 1, maximum: MAX_COMMAND_STEPS, description: 'Hard physics-step budget for the latched command.' },
        },
        required: ['schema_version', 'command', 'targets_rad'],
        additionalProperties: false,
      },
      {
        type: 'object',
        properties: {
          schema_version: version,
          command: { type: 'string', const: 'advance' },
          advance_seconds: { type: 'number', exclusiveMinimum: 0, maximum: MAX_ADVANCE_SECONDS },
        },
        required: ['schema_version', 'command', 'advance_seconds'],
        additionalProperties: false,
      },
    ],
    additionalProperties: false,
  };
}

export function getAsimovPhysicalControlDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'asimov' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'control_asimov_physical_simulation',
    title: 'Control the Asimov 1 physical MuJoCo simulation',
    description: `Control the active Asimov physical scene through ${WEBMCP_ASIMOV_SCHEMA_VERSION}. Bounded joint torques under the selected reference or experimental actuator profile, measured state, gravity and contacts. Estimated PD gains are not hardware calibration. Mounted scenes fix the pelvis explicitly. Experimental standing can be engaged only in its declared standing scene; acceptance is not achievement. Separate synthetic sensor reads are available only in actuator experiments. No walk, grasp, root-pose write or external-force operation is exposed.`,
    inputSchema: createAsimovControlSchema(),
  };
}

function parse(input) {
  plain(input);
  if (input.schema_version !== WEBMCP_ASIMOV_SCHEMA_VERSION) invalid(`schema_version must be ${WEBMCP_ASIMOV_SCHEMA_VERSION}.`);
  const command = String(input.command || '');
  if (['inspect_capability', 'read_state', 'read_sensors', 'engage_stand', 'stop', 'reset'].includes(command)) {
    onlyKeys(input, ['schema_version', 'command']);
    return { command };
  }
  const advanceOf = (value, { required = false } = {}) => {
    if (value == null) {
      if (required) invalid('advance_seconds is required for this command.');
      return 0;
    }
    const seconds = value;
    if (!Number.isFinite(seconds) || seconds < 0 || seconds > MAX_ADVANCE_SECONDS) invalid(`advance_seconds must be between 0 and ${MAX_ADVANCE_SECONDS}.`);
    if (required && seconds <= 0) invalid('advance_seconds must be greater than 0.');
    return seconds;
  };
  if (command === 'advance') {
    onlyKeys(input, ['schema_version', 'command', 'advance_seconds']);
    return { command, advanceSeconds: advanceOf(input.advance_seconds, { required: true }) };
  }
  if (command !== 'set_joint_targets') invalid('Asimov 1 physical command must be inspect_capability, read_state, read_sensors, engage_stand, set_joint_targets, advance, stop or reset.');
  onlyKeys(input, ['schema_version', 'command', 'targets_rad', 'advance_seconds', 'max_steps']);
  plain(input.targets_rad, 'targets_rad');
  const entries = Object.entries(input.targets_rad);
  if (!entries.length) invalid('targets_rad must contain at least one joint target.');
  const targetsRad = {};
  for (const [jointId, raw] of entries) {
    const range = JOINT_RANGES[jointId];
    if (!range) invalid(`Unknown Asimov 1 joint: ${jointId}.`);
    const value = raw;
    if (!Number.isFinite(value) || value < range[0] || value > range[1]) invalid(`${jointId} must be within ${range[0]}..${range[1]} radians.`);
    targetsRad[jointId] = value;
  }
  const steps = input.max_steps == null ? DEFAULT_COMMAND_STEPS : input.max_steps;
  if (!Number.isInteger(steps) || steps < 1 || steps > MAX_COMMAND_STEPS) invalid(`max_steps must be an integer from 1 to ${MAX_COMMAND_STEPS}.`);
  return { command, targetsRad, advanceSeconds: advanceOf(input.advance_seconds), maxSteps: steps };
}

function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }

function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'asimov' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready Asimov 1 physical workspace.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before direct Asimov 1 control.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'Asimov 1 PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }) });
}

function assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The Asimov 1 control call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'asimov' || context.simulationMode !== 'physical_mujoco' || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active Asimov 1 workspace or simulator changed during control.', { retryable: true });
  if (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The Asimov 1 PhysicsSession epoch changed during control.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during direct control.', { retryable: true });
}

function observedState(facade) {
  const state = facade.app.sim.getState?.();
  if (!state) return null;
  return {
    simulationTimeSeconds: state.simulation_time_s,
    root: state.root,
    footContacts: state.foot_contacts,
    contacts: state.contacts,
    controllerMode: state.controller_mode,
    actuationEnabled: state.actuation_enabled,
    joints: state.joints,
    walking: 'unsupported',
  };
}

export async function executeAsimovPhysicalControl(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-asimov-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const base = { ok: true, profileId: 'asimov', robot: PROFILES.asimov.label, schemaVersion: WEBMCP_ASIMOV_SCHEMA_VERSION, hardwareValidated: false };
  try {
    assertCurrent(facade, baseline, expectedEpoch, signal);

    if (parsed.command === 'inspect_capability') {
      // Read-only. It reports the capability table verbatim and starts nothing.
      return {
        ...base,
        command: 'inspect_capability',
        sourceRevision: ASIMOV_SOURCE.revision,
        rootMode: facade.app.sim.getState()?.root?.mode,
        capabilities: {jointTargets:'supported',walking:'unsupported',standing:facade.app.scenario?.physicalSceneId==='asimov-standing'?'experimental-flat-floor-trial':'unsupported',neck:'fixed in source'},
        actuatorModel:facade.app.sim.getState()?.actuator_model??null,
        limitations: [...(facade.app.scenario?.limitations??ASIMOV_LIMITATIONS)],
        physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
      };
    }

    if (parsed.command === 'read_state') {
      return { ...base, command: 'read_state', observedState: observedState(facade), taskEvaluation: facade.app.sim.getTaskEvaluation?.(), physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'read_sensors') {
      return {...base,command:'read_sensors',sensorObservation:facade.app.sim.getSensorObservation(),physicalAuthority:facade.app.sim.getPhysicalAuthorityToken?.()};
    }
    if (parsed.command === 'engage_stand') {
      if(facade.app.scenario?.physicalSceneId!=='asimov-standing') invalid('Standing is declared only in the experimental Standing Trial scene');
      await facade.app.sim.engageStand();
      assertCurrent(facade,baseline,expectedEpoch,signal);
      facade.app.renderPanels?.();
      return {...base,command:'engage_stand',note:'Controller engaged without advancing time; read the continuous standing assessment after bounded advances.',observedState:observedState(facade),physicalAuthority:facade.app.sim.getPhysicalAuthorityToken?.()};
    }

    if (parsed.command === 'reset') {
      if (!(await facade.app.resetSimulation())) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'Asimov 1 physical simulation could not be reset.', { retryable: true });
      assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange: true });
      // Reset establishes a new initial condition. It is never a recovery and never a success.
      return { ...base, command: 'reset', reset: true, note: 'reset establishes a new initial condition; it is not a recovery and is not task progress', physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'stop') {
      await facade.app.sim.stop();
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.setStatus?.('Agent commanded a bounded Asimov 1 hold at the measured joint state');
      facade.app.renderPanels?.();
      return { ...base, command: 'stop', observedState: observedState(facade), note: 'stop holds the measured joint state through the bounded controller; it does not teleport velocities to zero', physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    if (parsed.command === 'advance') {
      await facade.app.sim.advanceTime(parsed.advanceSeconds);
      assertCurrent(facade, baseline, expectedEpoch, signal);
      facade.app.renderPanels?.();
      return { ...base, command: 'advance', advanceSeconds: parsed.advanceSeconds, observedState: observedState(facade), taskEvaluation: facade.app.sim.getTaskEvaluation?.(), physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
    }

    const result = await facade.app.sim.applyPhysicalTargets(parsed.targetsRad, { maxSteps: parsed.maxSteps, advanceSeconds: parsed.advanceSeconds });
    assertCurrent(facade, baseline, expectedEpoch, signal);
    facade.app.setStatus?.('Agent applied bounded Asimov 1 joint targets');
    facade.app.renderPanels?.();
    return {
      ...base,
      command: 'set_joint_targets',
      commandStatus: result?.status ?? 'accepted',
      commandId: result?.commandId ?? null,
      requestedTargetsRad: structuredClone(parsed.targetsRad),
      advanceSeconds: parsed.advanceSeconds,
      observedState: observedState(facade),
      taskEvaluation: facade.app.sim.getTaskEvaluation?.(),
      note: 'requestedTargetsRad is a command, not a measurement; observedState.joints reports the accepted command and the measured position separately.',
      physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(),
    };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 280), { retryable: true, details: { profileId: 'asimov' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}

export const ASIMOV_CONTROL_LIMITS = Object.freeze({
  maxAdvanceSeconds: MAX_ADVANCE_SECONDS,
  maxCommandSteps: MAX_COMMAND_STEPS,
  defaultCommandSteps: DEFAULT_COMMAND_STEPS,
  exposedCommands: Object.freeze(['inspect_capability', 'read_state', 'read_sensors', 'engage_stand', 'set_joint_targets', 'advance', 'stop', 'reset']),
  // Named so a reviewer can grep for them: none of these exists as an agent-reachable operation.
  neverExposed: Object.freeze(['set_root_pose', 'set_root_velocity', 'force_upright', 'walk', 'set_actuation', 'apply_external_force', 'place_object']),
});
