import { OPENARM_V2_PHASE5A_SCENE } from './openarm-scene.js';

function finiteVec3(value) {
  return Array.isArray(value) && value.length === 3 && value.every((item) => Number.isFinite(Number(item)));
}
function norm3(value) { return finiteVec3(value) ? Math.hypot(...value.map(Number)) : Number.POSITIVE_INFINITY; }
function distance3(a, b) { return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1]), Number(a[2]) - Number(b[2])); }
function horizontalDistance(a, b) { return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1])); }
function contactPair(observation, first, second) {
  return (observation?.contacts || []).some((contact) => {
    const a = contact?.geom1Name;
    const b = contact?.geom2Name;
    return (a === first && b === second) || (a === second && b === first);
  });
}
function objectTouchesGeom(observation, objectGeoms, otherGeom) {
  return objectGeoms.some((objectGeom) => contactPair(observation, objectGeom, otherGeom));
}
function objectPosition(observation, objectId) {
  const value = observation?.bodies?.[objectId]?.positionM;
  return finiteVec3(value) ? value.map(Number) : null;
}
function bodyVelocity(observation, objectId, key) {
  const value = observation?.bodies?.[objectId]?.[key];
  return finiteVec3(value) ? value.map(Number) : null;
}

function newObjectState(spec) {
  return {
    spec: structuredClone(spec),
    initialPositionM: null,
    lastPositionM: null,
    currentInnerContact: false,
    currentOuterContact: false,
    currentGripperContact: false,
    currentSupportContact: false,
    innerContactSeen: false,
    outerContactSeen: false,
    graspSeen: false,
    liftSeen: false,
    carrySeen: false,
    releaseSeen: false,
    settled: false,
    retreated: false,
    carryAnchorPositionM: null,
    maxHeldHorizontalTravelM: 0,
    maxZM: Number.NEGATIVE_INFINITY,
    settleCandidatePositionM: null,
    settleCandidateTimeSeconds: null,
    settleDriftM: null,
    releaseTimeSeconds: null,
    settleTimeSeconds: null,
    retreatTimeSeconds: null,
    contactObservationCount: 0,
    supportObservationCount: 0,
  };
}

export class OpenArmBimanualStackEvaluator {
  constructor({ goal = OPENARM_V2_PHASE5A_SCENE.taskGoal } = {}) {
    this.goal = structuredClone(goal);
    this.reset();
  }

  reset(initialObservation = null) {
    this.objects = {
      flask: newObjectState(this.goal.flask),
      beaker: newObjectState(this.goal.beaker),
    };
    this.orderViolation = false;
    this.lastSimulationTimeSeconds = null;
    this.lastObservation = null;
    if (initialObservation) this.observe(initialObservation);
    return this.snapshot();
  }

  observe(observation) {
    const simulationTimeSeconds = Number(observation?.simulationTimeSeconds);
    if (!Number.isFinite(simulationTimeSeconds)) return this.snapshot();
    this.#observeObject('flask', observation, simulationTimeSeconds);
    this.#observeObject('beaker', observation, simulationTimeSeconds);

    const left = this.objects.flask;
    const right = this.objects.beaker;
    if ((right.graspSeen || right.liftSeen || right.carrySeen) && !left.retreated) this.orderViolation = true;

    this.lastSimulationTimeSeconds = simulationTimeSeconds;
    this.lastObservation = observation;
    return this.snapshot();
  }

  #observeObject(key, observation, simulationTimeSeconds) {
    const state = this.objects[key];
    const spec = state.spec;
    const positionM = objectPosition(observation, spec.objectId);
    if (!positionM) return;
    if (!state.initialPositionM) state.initialPositionM = [...positionM];
    state.lastPositionM = [...positionM];
    state.maxZM = Math.max(state.maxZM, positionM[2]);

    const inner = objectTouchesGeom(observation, spec.objectGeoms, spec.gripperGeoms[0]);
    const outer = objectTouchesGeom(observation, spec.objectGeoms, spec.gripperGeoms[1]);
    const gripperContact = inner || outer;
    const supportContact = objectTouchesGeom(observation, spec.objectGeoms, spec.supportGeom);
    state.currentInnerContact = inner;
    state.currentOuterContact = outer;
    state.currentGripperContact = gripperContact;
    state.currentSupportContact = supportContact;
    if (inner) state.innerContactSeen = true;
    if (outer) state.outerContactSeen = true;
    if (gripperContact) state.contactObservationCount += 1;
    if (supportContact) state.supportObservationCount += 1;
    if (state.innerContactSeen && state.outerContactSeen) state.graspSeen = true;

    const liftThreshold = Number(spec.initialBodyZM) + Number(this.goal.liftClearanceM);
    if (state.graspSeen && gripperContact && positionM[2] > liftThreshold) {
      state.liftSeen = true;
      if (!state.carrySeen && !state.carryAnchorPositionM) state.carryAnchorPositionM = [...positionM];
    }

    if (!state.carrySeen && state.carryAnchorPositionM) {
      if (!gripperContact) {
        state.carryAnchorPositionM = null;
        state.maxHeldHorizontalTravelM = 0;
      } else {
        const heldTravel = horizontalDistance(positionM, state.carryAnchorPositionM);
        state.maxHeldHorizontalTravelM = Math.max(state.maxHeldHorizontalTravelM, heldTravel);
        if (heldTravel >= Number(this.goal.carryHorizontalM)) state.carrySeen = true;
      }
    }

    const [targetX, targetY] = spec.targetCenterXYM;
    const [halfX, halfY] = spec.targetHalfExtentsXYM;
    const inTarget = Math.abs(positionM[0] - targetX) <= halfX && Math.abs(positionM[1] - targetY) <= halfY;
    const nearSupportHeight = Math.abs(positionM[2] - Number(spec.initialBodyZM)) <= 0.015;

    if (!state.releaseSeen && state.carrySeen && !gripperContact && supportContact && inTarget && nearSupportHeight) {
      state.releaseSeen = true;
      state.releaseTimeSeconds = simulationTimeSeconds;
    }

    const linearSpeed = norm3(bodyVelocity(observation, spec.objectId, 'linearVelocityMS'));
    const angularSpeed = norm3(bodyVelocity(observation, spec.objectId, 'angularVelocityRadS'));
    const settleEligible = state.releaseSeen
      && supportContact
      && !gripperContact
      && inTarget
      && nearSupportHeight
      && linearSpeed <= Number(this.goal.maxSettleLinearSpeedMS)
      && angularSpeed <= Number(this.goal.maxSettleAngularSpeedRadS);

    if (!settleEligible) {
      state.settleCandidatePositionM = null;
      state.settleCandidateTimeSeconds = null;
      state.settleDriftM = null;
      state.settled = false;
    } else if (!state.settleCandidatePositionM || state.settleCandidateTimeSeconds == null) {
      state.settleCandidatePositionM = [...positionM];
      state.settleCandidateTimeSeconds = simulationTimeSeconds;
      state.settleDriftM = 0;
    } else {
      const drift = distance3(positionM, state.settleCandidatePositionM);
      if (drift > Number(this.goal.maxSettleDriftM)) {
        state.settleCandidatePositionM = [...positionM];
        state.settleCandidateTimeSeconds = simulationTimeSeconds;
        state.settleDriftM = 0;
        state.settled = false;
      } else {
        state.settleDriftM = Math.max(Number(state.settleDriftM || 0), drift);
        if (simulationTimeSeconds - state.settleCandidateTimeSeconds + 1e-12 >= Number(this.goal.settleSeconds)) {
          state.settled = true;
          state.settleTimeSeconds ??= simulationTimeSeconds;
        }
      }
    }

    if (state.settled && !gripperContact) {
      const ee = observation?.bodies?.[`openarm_${spec.side}_ee_base_link`]?.positionM;
      if (finiteVec3(ee) && distance3(ee, positionM) >= Number(this.goal.retreatDistanceM)) {
        state.retreated = true;
        state.retreatTimeSeconds ??= simulationTimeSeconds;
      }
    }
  }

  snapshot() {
    const snapshotObject = (state) => Object.freeze({
      objectId: state.spec.objectId,
      side: state.spec.side,
      graspSeen: state.graspSeen,
      innerContactSeen: state.innerContactSeen,
      outerContactSeen: state.outerContactSeen,
      liftSeen: state.liftSeen,
      carrySeen: state.carrySeen,
      releaseSeen: state.releaseSeen,
      settled: state.settled,
      retreated: state.retreated,
      currentGripperContact: state.currentGripperContact,
      currentSupportContact: state.currentSupportContact,
      contactObservationCount: state.contactObservationCount,
      supportObservationCount: state.supportObservationCount,
      initialPositionM: state.initialPositionM ? [...state.initialPositionM] : null,
      finalPositionM: state.lastPositionM ? [...state.lastPositionM] : null,
      maxZM: Number.isFinite(state.maxZM) ? state.maxZM : null,
      maxHeldHorizontalTravelM: state.maxHeldHorizontalTravelM,
      settleEvidenceDurationSeconds: state.settleCandidateTimeSeconds == null || this.lastSimulationTimeSeconds == null
        ? 0 : Math.max(0, this.lastSimulationTimeSeconds - state.settleCandidateTimeSeconds),
      settleDriftM: state.settleDriftM,
      releaseTimeSeconds: state.releaseTimeSeconds,
      settleTimeSeconds: state.settleTimeSeconds,
      retreatTimeSeconds: state.retreatTimeSeconds,
    });
    const flask = snapshotObject(this.objects.flask);
    const beaker = snapshotObject(this.objects.beaker);
    const success = Boolean(
      !this.orderViolation
      && flask.graspSeen && flask.liftSeen && flask.carrySeen && flask.releaseSeen && flask.settled && flask.retreated
      && beaker.graspSeen && beaker.liftSeen && beaker.carrySeen && beaker.releaseSeen && beaker.settled && beaker.retreated
      && flask.currentSupportContact && beaker.currentSupportContact
      && !flask.currentGripperContact && !beaker.currentGripperContact
    );
    return Object.freeze({
      task: 'OpenArm V2 Physical Bimanual Heater and Ring-Stand Stack',
      physical: true,
      success,
      orderViolation: this.orderViolation,
      flask,
      beaker,
      evidence: 'MuJoCo body motion, free-joint velocities, named fingertip/vessel contacts, intended support contacts and simulation-time dwell. No object weld, parenting, snap, teleport or synthetic success event is used. Primitive collision surrogates and dry workcell parameters are simulator estimates, not hardware calibration.',
    });
  }
}
