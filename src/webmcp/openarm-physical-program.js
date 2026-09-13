import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from '../physics/openarm-model-package.js';
import { WebMcpDomainError } from './agent-facade.js';

export const WEBMCP_OPENARM_PROGRAM_SCHEMA_VERSION = 'robobuddy.openarm.program.v1';
const MAX_SEGMENTS = 24;
const MAX_SEGMENT_SECONDS = 2;
const MAX_TOTAL_SECONDS = 12;
const TIMESTEP_SECONDS = Number(OPENARM_V2_PHASE5A_MODEL_PACKAGE.physics.timestepSeconds);
const ALIGNMENT_TOLERANCE = 1e-9;
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
function targetsSchema() {
  return {
    type: 'object',
    properties: Object.fromEntries(Object.entries(RANGES).map(([jointId, range]) => [jointId, { type: 'number', minimum: range[0], maximum: range[1], description: 'radians; OpenArm V2 MuJoCo position target' }])),
    minProperties: 1,
    additionalProperties: false,
  };
}
export function createOpenArmPhysicalProgramSchema() {
  return {
    type: 'object',
    properties: {
      schema_version: { type: 'string', const: WEBMCP_OPENARM_PROGRAM_SCHEMA_VERSION },
      segments: {
        type: 'array', minItems: 1, maxItems: MAX_SEGMENTS,
        items: {
          type: 'object',
          properties: {
            name: { type: 'string', minLength: 1, maxLength: 80 },
            targets_rad: targetsSchema(),
            duration_seconds: { type: 'number', minimum: TIMESTEP_SECONDS, maximum: MAX_SEGMENT_SECONDS, description: `Simulation seconds; must align to ${TIMESTEP_SECONDS} s MuJoCo steps.` },
          },
          required: ['targets_rad', 'duration_seconds'], additionalProperties: false,
        },
      },
    },
    required: ['schema_version', 'segments'],
    additionalProperties: false,
  };
}
export function getOpenArmPhysicalProgramDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'program_openarm_simulation',
    title: 'Run a bounded OpenArm physical motion program',
    description: `Execute up to ${MAX_SEGMENTS} joint-target segments (${MAX_TOTAL_SECONDS} s total) against the same authoritative OpenArm MuJoCo PhysicsSession. Every target is radians, every duration is simulation time, and each segment has an exact physics-step budget. The tool does not move objects directly, attach grasps, save source, publish, or control hardware.`,
    inputSchema: createOpenArmPhysicalProgramSchema(),
  };
}
function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }
function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready OpenArm V2 physical workspace.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before direct OpenArm programming.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'OpenArm PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }) });
}
function assertCurrent(facade, baseline, expectedEpoch, signal) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The OpenArm motion program was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco' || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active OpenArm workspace or simulator changed during the motion program.', { retryable: true });
  if (!sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The OpenArm PhysicsSession epoch changed during the motion program.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during the motion program.', { retryable: true });
}
function parse(input) {
  if (!input || typeof input !== 'object' || Array.isArray(input)) invalid('input must be an object.');
  for (const key of Object.keys(input)) if (!['schema_version', 'segments'].includes(key)) invalid(`Unexpected input field: ${key}.`);
  if (input.schema_version !== WEBMCP_OPENARM_PROGRAM_SCHEMA_VERSION) invalid(`schema_version must be ${WEBMCP_OPENARM_PROGRAM_SCHEMA_VERSION}.`);
  if (!Array.isArray(input.segments) || !input.segments.length || input.segments.length > MAX_SEGMENTS) invalid(`segments must contain 1..${MAX_SEGMENTS} entries.`);
  let total = 0;
  const segments = input.segments.map((raw, index) => {
    if (!raw || typeof raw !== 'object' || Array.isArray(raw)) invalid(`segments[${index}] must be an object.`);
    for (const key of Object.keys(raw)) if (!['name', 'targets_rad', 'duration_seconds'].includes(key)) invalid(`Unexpected segments[${index}] field: ${key}.`);
    if (!raw.targets_rad || typeof raw.targets_rad !== 'object' || Array.isArray(raw.targets_rad) || !Object.keys(raw.targets_rad).length) invalid(`segments[${index}].targets_rad must be a non-empty object.`);
    const targetsRad = {};
    for (const [jointId, valueRaw] of Object.entries(raw.targets_rad)) {
      const range = RANGES[jointId];
      if (!range) invalid(`Unknown OpenArm V2 physical joint in segment ${index}: ${jointId}.`);
      const value = Number(valueRaw);
      if (!Number.isFinite(value) || value < range[0] || value > range[1]) invalid(`${jointId} in segment ${index} must be within ${range[0]}..${range[1]} radians.`);
      targetsRad[jointId] = value;
    }
    const durationSeconds = Number(raw.duration_seconds);
    if (!Number.isFinite(durationSeconds) || durationSeconds < TIMESTEP_SECONDS || durationSeconds > MAX_SEGMENT_SECONDS) invalid(`segments[${index}].duration_seconds must be ${TIMESTEP_SECONDS}..${MAX_SEGMENT_SECONDS}.`);
    const steps = Math.round(durationSeconds / TIMESTEP_SECONDS);
    if (Math.abs(durationSeconds - steps * TIMESTEP_SECONDS) > ALIGNMENT_TOLERANCE) invalid(`segments[${index}].duration_seconds must align to ${TIMESTEP_SECONDS} s MuJoCo steps.`);
    total += durationSeconds;
    if (total > MAX_TOTAL_SECONDS + ALIGNMENT_TOLERANCE) invalid(`Motion program exceeds the ${MAX_TOTAL_SECONDS} s total simulation-time limit.`);
    return Object.freeze({ name: raw.name ? String(raw.name) : `segment_${index + 1}`, targetsRad: Object.freeze(targetsRad), durationSeconds, steps });
  });
  return Object.freeze({ segments: Object.freeze(segments), totalSeconds: total });
}
function compactObservation(observation) {
  return {
    simulationTimeSeconds: Number(observation?.simulationTimeSeconds || 0),
    jointsRad: Object.fromEntries(Object.entries(observation?.joints || {}).map(([id, state]) => [id, Number(state.positionRad)])),
    contactCount: Number(observation?.contactCount || 0),
  };
}
export async function executeOpenArmPhysicalProgram(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-openarm-program-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  const results = [];
  try {
    for (const [index, segment] of parsed.segments.entries()) {
      assertCurrent(facade, baseline, expectedEpoch, signal);
      const result = await facade.app.sim.applyPhysicalTargets(segment.targetsRad, { maxSteps: segment.steps, advanceSeconds: segment.durationSeconds });
      assertCurrent(facade, baseline, expectedEpoch, signal);
      results.push({ index, name: segment.name, durationSeconds: segment.durationSeconds, acceptedTargetsRad: result.acceptedTargetsRad, observedState: compactObservation(result.observation), taskEvaluation: result.taskEvaluation });
    }
    facade.app.setStatus?.(`Agent completed ${results.length}-segment OpenArm physical motion program`);
    facade.app.renderPanels?.();
    return { ok: true, profileId: 'openarm', schemaVersion: WEBMCP_OPENARM_PROGRAM_SCHEMA_VERSION, hardwareValidated: false, totalSimulationSeconds: parsed.totalSeconds, segments: results, taskEvaluation: facade.app.sim.getTaskEvaluation?.(), physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.() };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 360), { retryable: true, details: { completedSegments: results.length, profileId: 'openarm' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}
