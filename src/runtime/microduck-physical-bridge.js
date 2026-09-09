import { LIVE_SIM_API_VERSION } from './live-python-bridge.js';
import { MICRODUCK_MAX_ADVANCE_SECONDS } from '../physics/microduck-physical-simulator.js';
import { MICRODUCK_CONTROL_INTERVAL_SECONDS, MICRODUCK_POLICY_JOINT_ORDER } from '../physics/microduck-controller.js';
import { MICRODUCK_CAPABILITY_AUDIT, microduckCapability } from '../physics/microduck-capabilities.js';
import { MICRODUCK_COMMAND_LIMITS, MICRODUCK_GAIT_ONSET_MS } from '../physics/microduck-scene.js';

// The live-Python bridge for the MicroDuck physical workspace.
//
// It implements the same boundary methods PhysicalPythonRuntime already dispatches, so the
// worker, the run epoch, pause/step/cancel and the stale-bridge rejection are all reused
// unchanged. What differs is the command surface: a learner program does NOT write joint
// targets here. It issues the same bounded velocity/head/body command the deployed robot
// takes, and the policy plus the plant decide what happens.
//
// Nothing reachable from Python can write root pose, root velocity, ball state or a task
// outcome. `get_observation` deliberately separates the requested command, the controller's
// decision and the actual physical state, so a program cannot mistake one for another.

const COMMAND_FIELDS = Object.freeze(['vx', 'vy', 'vyaw', 'neckPitch', 'headPitch', 'headYaw', 'headRoll', 'bodyZ', 'bodyRoll', 'bodyPitch']);
const SKILL_IDS = Object.freeze(MICRODUCK_CAPABILITY_AUDIT.map((item) => item.id));

function liveError(code, message, details = null) {
  const error = new Error(message);
  error.code = code;
  if (details) error.details = details;
  return error;
}

function finiteNonNegative(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number) || number < 0) throw new RangeError(`${label} must be a finite non-negative number`);
  return number;
}

export class MicroDuckPhysicalBridge {
  constructor(simulator) {
    if (!simulator) throw new TypeError('MicroDuckPhysicalBridge requires the physical simulator');
    this.simulator = simulator;
    this.connected = false;
    this.disposed = false;
    this.cancelled = false;
    this.owner = null;
  }

  async connect(expectedRobotId = null) {
    this.#assertLive();
    const authority = this.simulator.getPhysicalAuthorityToken?.();
    if (!authority?.sessionId) throw liveError('SIMULATION_NOT_READY', 'MicroDuck PhysicsSession authority is unavailable');
    if (expectedRobotId && authority.robotId !== expectedRobotId) {
      throw liveError('PROFILE_MISMATCH', `Physical session robot ${authority.robotId} does not match requested ${expectedRobotId}`);
    }
    this.owner = { ...authority, runEpoch: this.simulator.runEpoch };
    this.connected = true;
    this.cancelled = false;
    return Object.freeze({
      apiVersion: LIVE_SIM_API_VERSION,
      physical: true,
      ...structuredClone(this.owner),
      backend: 'browser-mujoco',
      commandFields: [...COMMAND_FIELDS],
      commandLimits: structuredClone(MICRODUCK_COMMAND_LIMITS),
      gaitOnsetMS: MICRODUCK_GAIT_ONSET_MS,
      controlIntervalSeconds: MICRODUCK_CONTROL_INTERVAL_SECONDS,
      policyJoints: [...MICRODUCK_POLICY_JOINT_ORDER],
      capabilities: MICRODUCK_CAPABILITY_AUDIT.map((item) => ({ id: item.id, status: item.status, physical: Boolean(item.physicalPolicy) })),
      units: Object.freeze({ length: 'm', time: 's', angle: 'rad', linearVelocity: 'm/s', angularVelocity: 'rad/s' }),
      observationProfiles: Object.freeze(['ground_truth']),
      note: 'A command is a request into a trained policy and a MuJoCo plant. Nothing here writes root pose, root velocity, ball state or task success.',
    });
  }

  /**
   * Latch a bounded command, optionally requesting a skill, and optionally advance.
   *
   * `action` is a plain object of command fields, with an optional `skill`. Unknown fields
   * and unsupported skills are rejected rather than ignored.
   */
  async sendAction(action, { maxSteps = null } = {}) {
    this.#assertOwner();
    if (!action || typeof action !== 'object' || Array.isArray(action)) throw liveError('INVALID_ARGUMENT', 'send_action requires a command object');
    const request = {};
    let skill = null;
    let advanceSeconds = 0;
    for (const [key, raw] of Object.entries(action)) {
      if (key === 'skill') {
        skill = String(raw);
        if (!SKILL_IDS.includes(skill)) throw liveError('INVALID_ARGUMENT', `Unknown MicroDuck capability: ${skill}`);
        continue;
      }
      if (key === 'advance_seconds') { advanceSeconds = finiteNonNegative(raw, 'advance_seconds'); continue; }
      if (!COMMAND_FIELDS.includes(key)) throw liveError('INVALID_ARGUMENT', `Unknown MicroDuck command field: ${key}`);
      const value = Number(raw);
      if (!Number.isFinite(value)) throw liveError('INVALID_ARGUMENT', `${key} must be finite`);
      request[key] = value;
    }
    if (advanceSeconds > MICRODUCK_MAX_ADVANCE_SECONDS) throw new RangeError(`advance_seconds is bounded to ${MICRODUCK_MAX_ADVANCE_SECONDS} simulated seconds per call`);
    void maxSteps;

    const accepted = Object.keys(request).length ? this.simulator.setCommand(request) : null;
    let skillResult = null;
    if (skill) {
      skillResult = this.simulator.requestSkill(skill);
      if (!skillResult.accepted) {
        const capability = microduckCapability(skill);
        throw liveError('CAPABILITY_UNSUPPORTED', `${skill} is unsupported in MicroDuck physical mode: ${skillResult.reason}`, {
          capability: skill, status: capability?.status ?? 'unsupported in physical mode', routedToLegacy: false,
        });
      }
    }
    let run = null;
    if (advanceSeconds > 0) run = await this.simulator.advanceSeconds(advanceSeconds);
    return {
      apiVersion: LIVE_SIM_API_VERSION,
      status: 'accepted',
      requested: accepted ? accepted.requested : structuredClone(this.simulator.getState().requested.command),
      limitedBy: accepted ? accepted.limitedBy : [],
      skill: skill ? { id: skill, status: skillResult.status, capability: skillResult.capability } : null,
      advanceSeconds,
      executedTicks: run?.executedTicks ?? 0,
      completed: run ? run.completed : null,
      note: 'Accepting a command is not achieving it. Read achieved motion from get_observation().',
    };
  }

  /** Advance bounded simulation time through the fixed-cadence controller loop. */
  async advance(seconds) {
    this.#assertOwner();
    const duration = finiteNonNegative(seconds, 'advance duration');
    if (duration === 0) return this.getObservation();
    if (duration > MICRODUCK_MAX_ADVANCE_SECONDS) throw new RangeError(`advance duration is bounded to ${MICRODUCK_MAX_ADVANCE_SECONDS} simulated seconds per call`);
    const run = await this.simulator.advanceSeconds(duration);
    return { ...this.getObservation(), executedTicks: run.executedTicks, completed: run.completed, cancelled: run.cancelled };
  }

  /**
   * The three views, deliberately separate: what was requested, what the controller decided,
   * and what the physics actually is. Only `actual` is a measurement.
   */
  getObservation({ view = 'ground_truth' } = {}) {
    this.#assertOwner();
    if (view !== 'ground_truth') throw liveError('INVALID_ARGUMENT', `Observation view ${view} is unsupported by the physical preview`);
    const state = this.simulator.getState();
    return {
      apiVersion: LIVE_SIM_API_VERSION,
      view,
      requested: state.requested,
      controller: state.controller,
      actual: state.actual,
      taskEvaluation: this.simulator.report(),
      capabilities: state.capabilities,
      hardwareValidated: false,
    };
  }

  /**
   * Wait until the trunk settles, or until the bounded timeout. It reports what actually
   * happened; it never asserts success.
   */
  async waitForGoal(goal = {}, { timeoutSeconds = 2, controllerPeriodSeconds = MICRODUCK_CONTROL_INTERVAL_SECONDS } = {}) {
    this.#assertOwner();
    const timeout = Math.min(finiteNonNegative(timeoutSeconds, 'timeout_seconds'), MICRODUCK_MAX_ADVANCE_SECONDS);
    const period = Math.max(MICRODUCK_CONTROL_INTERVAL_SECONDS, finiteNonNegative(controllerPeriodSeconds, 'controller_period_seconds'));
    const upright = goal?.upright !== false;
    const deadlineTicks = Math.max(1, Math.round(timeout / period));
    let reached = false;
    for (let tick = 0; tick < deadlineTicks; tick += 1) {
      await this.simulator.advanceSeconds(period);
      const report = this.simulator.report();
      if (upright && report?.upright) { reached = true; break; }
      if (this.simulator.cancelled) break;
    }
    return { ...this.getObservation(), reached, timedOut: !reached, timeoutSeconds: timeout };
  }

  async pause() { this.#assertOwner(); await this.simulator.pause(); return { paused: true }; }
  async resume() { this.#assertOwner(); await this.simulator.resume(); return { resumed: true }; }

  /** A reset establishes a new initial condition. It is never a recovery and never a success. */
  async reset() {
    this.#assertOwner();
    await this.simulator.reset();
    this.owner = { ...this.simulator.getPhysicalAuthorityToken(), runEpoch: this.simulator.runEpoch };
    return { ...this.getObservation(), reset: true, countsAsRecovery: false };
  }

  async cancel(reason = 'cancelled') {
    if (!this.connected) return false;
    this.cancelled = true;
    this.simulator.cancel(reason);
    this.connected = false;
    this.owner = null;
    return { cancelled: true, reason: String(reason) };
  }

  disconnect() { this.connected = false; this.owner = null; return { disconnected: true }; }
  dispose() { this.disposed = true; this.connected = false; this.owner = null; }

  #assertLive() { if (this.disposed) throw liveError('SIMULATION_NOT_READY', 'MicroDuck physical bridge is disposed'); }

  // A bridge from a replaced workspace or a cancelled run must never reach the new session.
  #assertOwner() {
    this.#assertLive();
    if (!this.connected || !this.owner) throw liveError('SIMULATION_NOT_READY', 'MicroDuck physical bridge is not connected');
    if (this.cancelled) throw liveError('OPERATION_CANCELLED', 'This MicroDuck run was cancelled');
    const authority = this.simulator.getPhysicalAuthorityToken?.();
    if (!authority?.sessionId) throw liveError('SIMULATION_NOT_READY', 'MicroDuck PhysicsSession authority is unavailable');
    if (authority.sessionId !== this.owner.sessionId || authority.sceneRevision !== this.owner.sceneRevision || authority.robotId !== this.owner.robotId) {
      throw liveError('OPERATION_CANCELLED', 'The MicroDuck physical session was replaced during this run');
    }
  }
}
