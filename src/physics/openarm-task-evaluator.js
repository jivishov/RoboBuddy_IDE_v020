import { OPENARM_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';
import { OPENARM_PHASE5A_SCENE } from './openarm-scene.js';

const BENCH = OPENARM_PHASE5A_MODEL_PACKAGE.benchmark;
const HALF_Z = BENCH.objectHalfHeightM;
const LIFT_CLEARANCE_M = 0.025;
const HELD_TRAVEL_M = 0.045;
const REST_Z_TOLERANCE_M = 0.012;
const REST_MIN_SECONDS = 0.20;
const REST_MAX_POSITION_DRIFT_M = 0.001;
const REST_MAX_LINEAR_SPEED_M_S = 0.03;
const REST_MAX_ANGULAR_SPEED_RAD_S = 0.8;
const RETREAT_DISTANCE_INCREASE_M = 0.025;

function finiteVec(value, size) {
  return Array.isArray(value) && value.length === size && value.every((item) => Number.isFinite(Number(item)));
}
function vec3(value) { return finiteVec(value, 3) ? value.map(Number) : null; }
function norm(value) { return Math.hypot(...value.map(Number)); }
function distance3(a, b) { return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1]), Number(a[2]) - Number(b[2])); }
function horizontalDistance(a, b) { return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1])); }
function hasPair(observation, first, second) {
  return (observation?.contacts || []).some((contact) => {
    const a = contact?.geom1Name;
    const b = contact?.geom2Name;
    return (a === first && b === second) || (a === second && b === first);
  });
}

function objectDefinition(key) {
  const def = BENCH[key];
  return {
    key,
    id: def.id,
    geom: def.geom,
    sourcePositionM: [...def.sourcePositionM],
    targetCenterXYM: [...def.targetCenterXYM],
    supportGeom: def.supportGeom,
    supportTopZM: Number(def.supportTopZM),
    fingerGeoms: [...def.fingerGeoms],
    eeBody: def.eeBody,
  };
}

function freshObject(def) {
  return {
    def,
    initialPositionM: null,
    lastPositionM: null,
    carryAnchorPositionM: null,
    releasePositionM: null,
    releaseEeDistanceM: null,
    restCandidatePositionM: null,
    restCandidateTimeSeconds: null,
    maxZM: Number.NEGATIVE_INFINITY,
    maxHeldTravelM: 0,
    contactObservationCount: 0,
    heldObservationCount: 0,
    supportObservationCount: 0,
    contactSeen: false,
    liftSeen: false,
    carrySeen: false,
    releaseSeen: false,
    supportSeen: false,
    settleSeen: false,
    retreatSeen: false,
    inTarget: false,
    currentGripperContact: false,
    currentSupportContact: false,
    linearSpeedMPerS: null,
    angularSpeedRadPerS: null,
    settleEvidenceDurationSeconds: 0,
    settlePositionDriftM: null,
  };
}

export function hasOpenArmObjectGripperContact(observation, objectKey) {
  const def = BENCH[objectKey];
  if (!def) return false;
  return def.fingerGeoms.some((finger) => hasPair(observation, def.geom, finger));
}

export function hasOpenArmObjectSupportContact(observation, objectKey) {
  const def = BENCH[objectKey];
  return Boolean(def && hasPair(observation, def.geom, def.supportGeom));
}

export class OpenArmPhase5AEvaluator {
  constructor({ goal = OPENARM_PHASE5A_SCENE.taskGoal } = {}) {
    this.goal = structuredClone(goal);
    this.reset();
  }

  reset(initialObservation = null) {
    this.flask = freshObject(objectDefinition('flask'));
    this.beaker = freshObject(objectDefinition('beaker'));
    this.lastSimulationTimeSeconds = null;
    this.orderViolation = false;
    this.lastObservation = null;
    if (initialObservation) this.observe(initialObservation);
    return this.snapshot();
  }

  observe(observation) {
    const time = Number(observation?.simulationTimeSeconds);
    if (!Number.isFinite(time)) return this.snapshot();
    this.#observeObject(this.flask, observation, time, true);
    const leftDone = this.flask.settleSeen && this.flask.retreatSeen;
    const rightContactNow = hasOpenArmObjectGripperContact(observation, 'beaker');
    if (!leftDone && rightContactNow) this.orderViolation = true;
    if (leftDone || this.beaker.contactSeen) this.#observeObject(this.beaker, observation, time, leftDone);
    this.lastSimulationTimeSeconds = time;
    this.lastObservation = observation;
    return this.snapshot();
  }

  #observeObject(state, observation, time, enabled) {
    const body = observation?.bodies?.[state.def.id];
    const positionM = vec3(body?.positionM);
    if (!positionM) return;
    if (!state.initialPositionM) state.initialPositionM = [...positionM];
    state.lastPositionM = [...positionM];
    state.maxZM = Math.max(state.maxZM, positionM[2]);

    const gripperContact = hasOpenArmObjectGripperContact(observation, state.def.key);
    const supportContact = hasOpenArmObjectSupportContact(observation, state.def.key);
    state.currentGripperContact = gripperContact;
    state.currentSupportContact = supportContact;
    if (gripperContact) state.contactObservationCount += 1;
    if (supportContact) state.supportObservationCount += 1;

    const linearVelocity = vec3(body?.linearVelocityMPerS);
    const angularVelocity = vec3(body?.angularVelocityRadPerS);
    state.linearSpeedMPerS = linearVelocity ? norm(linearVelocity) : null;
    state.angularSpeedRadPerS = angularVelocity ? norm(angularVelocity) : null;

    const dx = Math.abs(positionM[0] - state.def.targetCenterXYM[0]);
    const dy = Math.abs(positionM[1] - state.def.targetCenterXYM[1]);
    state.inTarget = dx <= BENCH.targetHalfExtentsXYM[0] && dy <= BENCH.targetHalfExtentsXYM[1];

    if (!enabled) return;
    if (gripperContact) state.contactSeen = true;
    const liftThreshold = state.def.sourcePositionM[2] + LIFT_CLEARANCE_M;
    if (gripperContact && positionM[2] > liftThreshold) {
      state.liftSeen = true;
      if (!state.carryAnchorPositionM) state.carryAnchorPositionM = [...positionM];
    }
    if (state.carryAnchorPositionM && !state.carrySeen) {
      if (!gripperContact) {
        state.carryAnchorPositionM = null;
        state.maxHeldTravelM = 0;
      } else {
        const travel = horizontalDistance(positionM, state.carryAnchorPositionM);
        state.maxHeldTravelM = Math.max(state.maxHeldTravelM, travel);
        state.heldObservationCount += 1;
        if (travel >= HELD_TRAVEL_M) state.carrySeen = true;
      }
    } else if (state.carrySeen && gripperContact) {
      state.heldObservationCount += 1;
    }

    const eePosition = vec3(observation?.bodies?.[state.def.eeBody]?.positionM);
    if (state.carrySeen && !gripperContact && !state.releaseSeen) {
      state.releaseSeen = true;
      state.releasePositionM = [...positionM];
      if (eePosition) state.releaseEeDistanceM = distance3(eePosition, positionM);
    }
    if (state.releaseSeen && supportContact) state.supportSeen = true;

    const expectedCenterZ = state.def.supportTopZM + HALF_Z;
    const nearSupport = Math.abs(positionM[2] - expectedCenterZ) <= REST_Z_TOLERANCE_M;
    const speedsAcceptable = state.linearSpeedMPerS != null && state.angularSpeedRadPerS != null
      && state.linearSpeedMPerS <= REST_MAX_LINEAR_SPEED_M_S
      && state.angularSpeedRadPerS <= REST_MAX_ANGULAR_SPEED_RAD_S;
    const restEligible = state.releaseSeen && state.inTarget && supportContact && nearSupport && !gripperContact && speedsAcceptable;
    if (!restEligible) {
      state.restCandidatePositionM = null;
      state.restCandidateTimeSeconds = null;
      state.settlePositionDriftM = null;
      state.settleEvidenceDurationSeconds = 0;
      state.settleSeen = false;
    } else if (state.restCandidateTimeSeconds == null || !state.restCandidatePositionM) {
      state.restCandidatePositionM = [...positionM];
      state.restCandidateTimeSeconds = time;
      state.settlePositionDriftM = 0;
      state.settleEvidenceDurationSeconds = 0;
    } else {
      const drift = distance3(positionM, state.restCandidatePositionM);
      if (drift > REST_MAX_POSITION_DRIFT_M) {
        state.restCandidatePositionM = [...positionM];
        state.restCandidateTimeSeconds = time;
        state.settlePositionDriftM = 0;
        state.settleEvidenceDurationSeconds = 0;
        state.settleSeen = false;
      } else {
        state.settlePositionDriftM = Math.max(Number(state.settlePositionDriftM || 0), drift);
        state.settleEvidenceDurationSeconds = Math.max(0, time - state.restCandidateTimeSeconds);
        state.settleSeen = state.settleEvidenceDurationSeconds + 1e-12 >= REST_MIN_SECONDS;
      }
    }

    if (state.releaseSeen && state.settleSeen && eePosition && state.releaseEeDistanceM != null) {
      state.retreatSeen = distance3(eePosition, positionM) >= state.releaseEeDistanceM + RETREAT_DISTANCE_INCREASE_M;
    }
  }

  snapshotObject(state) {
    return Object.freeze({
      id: state.def.id,
      contactSeen: state.contactSeen,
      liftSeen: state.liftSeen,
      carrySeen: state.carrySeen,
      releaseSeen: state.releaseSeen,
      supportSeen: state.supportSeen,
      settleSeen: state.settleSeen,
      retreatSeen: state.retreatSeen,
      inTarget: state.inTarget,
      currentGripperContact: state.currentGripperContact,
      currentSupportContact: state.currentSupportContact,
      contactObservationCount: state.contactObservationCount,
      heldObservationCount: state.heldObservationCount,
      supportObservationCount: state.supportObservationCount,
      maxZM: Number.isFinite(state.maxZM) ? state.maxZM : null,
      maxHeldTravelM: state.maxHeldTravelM,
      linearSpeedMPerS: state.linearSpeedMPerS,
      angularSpeedRadPerS: state.angularSpeedRadPerS,
      settleEvidenceDurationSeconds: state.settleEvidenceDurationSeconds,
      settlePositionDriftM: state.settlePositionDriftM,
      finalPositionM: state.lastPositionM ? [...state.lastPositionM] : null,
      targetCenterXYM: [...state.def.targetCenterXYM],
      supportTopZM: state.def.supportTopZM,
    });
  }

  snapshot() {
    const flask = this.snapshotObject(this.flask);
    const beaker = this.snapshotObject(this.beaker);
    const objectSucceeded = (state) => state.contactSeen && state.liftSeen && state.carrySeen && state.releaseSeen
      && state.supportSeen && state.settleSeen && state.retreatSeen && state.inTarget
      && state.currentSupportContact && !state.currentGripperContact;
    const success = !this.orderViolation && objectSucceeded(flask) && objectSucceeded(beaker);
    return Object.freeze({
      task: 'OpenArm V2 Physical Bimanual Heater and Ring-Stand Stack',
      physical: true,
      success,
      orderViolation: this.orderViolation,
      flask,
      beaker,
      evidence: `MuJoCo ground-truth body poses/velocities and named geometry contacts. Each object must be contacted by its own physical gripper, lifted while held, transported at least ${HELD_TRAVEL_M} m while held, released onto the intended support, remain inside the target region with linear speed <= ${REST_MAX_LINEAR_SPEED_M_S} m/s and angular speed <= ${REST_MAX_ANGULAR_SPEED_RAD_S} rad/s for ${REST_MIN_SECONDS} s with <= ${REST_MAX_POSITION_DRIFT_M} m drift, and the gripper must retreat. The beaker sequence cannot begin before the flask is settled and retreated. No task weld, attachment, snap, transform overwrite, or synthetic success event is accepted.`,
    });
  }
}
