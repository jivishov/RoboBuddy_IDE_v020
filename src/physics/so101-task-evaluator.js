import { SO101_MANIPULATION_MODEL_PACKAGE } from './model-packages.js';
import { SO101_MANIPULATION_SCENE } from './so101-scene.js';

const SUPPORT_Z_M = SO101_MANIPULATION_MODEL_PACKAGE.benchmark.workSurface.topZM;
const BLOCK_HALF_Z_M = SO101_MANIPULATION_MODEL_PACKAGE.benchmark.object.dimensionsM[2] / 2;
const LIFT_CLEARANCE_M = 0.030;
const CARRY_HORIZONTAL_M = 0.050;
const REST_Z_TOLERANCE_M = 0.012;
const REST_MIN_SETTLE_SECONDS = 0.20;
const GRIPPER_GEOM_RE = /^(fixed_jaw_|moving_jaw_)/;

function finiteVec3(value) {
  return Array.isArray(value) && value.length === 3 && value.every((item) => Number.isFinite(Number(item)));
}

function horizontalDistance(a, b) {
  return Math.hypot(Number(a[0]) - Number(b[0]), Number(a[1]) - Number(b[1]));
}

function hasNamedPair(observation, first, second) {
  return (observation?.contacts || []).some((contact) => {
    const a = contact?.geom1Name;
    const b = contact?.geom2Name;
    return (a === first && b === second) || (a === second && b === first);
  });
}

export function hasSo101BlockGripperContact(observation) {
  return (observation?.contacts || []).some((contact) => {
    const a = contact?.geom1Name;
    const b = contact?.geom2Name;
    return (a === 'benchmark_block_geom' && GRIPPER_GEOM_RE.test(b || ''))
      || (b === 'benchmark_block_geom' && GRIPPER_GEOM_RE.test(a || ''));
  });
}

export function hasSo101BlockTargetSupportContact(observation) {
  return hasNamedPair(observation, 'benchmark_block_geom', 'benchmark_target_support');
}

function blockPosition(observation) {
  const position = observation?.bodies?.benchmark_block?.positionM;
  return finiteVec3(position) ? position.map(Number) : null;
}

export class So101BlockTransferEvaluator {
  constructor({ goal = SO101_MANIPULATION_SCENE.taskGoal } = {}) {
    this.goal = structuredClone(goal);
    this.reset();
  }

  reset(initialObservation = null) {
    this.initialPositionM = null;
    this.lastPositionM = null;
    this.lastSimulationTimeSeconds = null;
    this.firstReleasedPositionM = null;
    this.firstReleasedTimeSeconds = null;
    this.maxBlockZM = Number.NEGATIVE_INFINITY;
    this.maxHorizontalTravelM = 0;
    this.contactObservationCount = 0;
    this.carriedContactObservationCount = 0;
    this.targetSupportContactObservationCount = 0;
    this.contactSeen = false;
    this.liftSeen = false;
    this.carrySeen = false;
    this.releaseSeen = false;
    this.settleSeen = false;
    this.inTarget = false;
    this.currentGripperContact = false;
    this.currentTargetSupportContact = false;
    this.lastObservation = null;
    if (initialObservation) this.observe(initialObservation);
    return this.snapshot();
  }

  observe(observation) {
    const positionM = blockPosition(observation);
    const simulationTimeSeconds = Number(observation?.simulationTimeSeconds);
    if (!positionM || !Number.isFinite(simulationTimeSeconds)) return this.snapshot();

    if (!this.initialPositionM) this.initialPositionM = [...positionM];
    this.maxBlockZM = Math.max(this.maxBlockZM, positionM[2]);
    this.maxHorizontalTravelM = Math.max(this.maxHorizontalTravelM, horizontalDistance(positionM, this.initialPositionM));

    const gripperContact = hasSo101BlockGripperContact(observation);
    const targetSupportContact = hasSo101BlockTargetSupportContact(observation);
    this.currentGripperContact = gripperContact;
    this.currentTargetSupportContact = targetSupportContact;
    if (gripperContact) {
      this.contactSeen = true;
      this.contactObservationCount += 1;
    }
    if (targetSupportContact) this.targetSupportContactObservationCount += 1;

    const liftThresholdM = SUPPORT_Z_M + BLOCK_HALF_Z_M + LIFT_CLEARANCE_M;
    if (this.contactSeen && positionM[2] > liftThresholdM) this.liftSeen = true;
    if (gripperContact && this.liftSeen && this.maxHorizontalTravelM > CARRY_HORIZONTAL_M) {
      this.carrySeen = true;
      this.carriedContactObservationCount += 1;
    }

    const [targetX, targetY] = this.goal.targetCenterXYM;
    const [halfX, halfY] = this.goal.targetHalfExtentsXYM;
    this.inTarget = Math.abs(positionM[0] - targetX) <= halfX && Math.abs(positionM[1] - targetY) <= halfY;

    if (this.carrySeen && this.contactSeen && !gripperContact && !this.releaseSeen) {
      this.releaseSeen = true;
      this.firstReleasedPositionM = [...positionM];
      this.firstReleasedTimeSeconds = simulationTimeSeconds;
    }

    if (this.releaseSeen && this.firstReleasedTimeSeconds != null) {
      const elapsedSinceRelease = simulationTimeSeconds - this.firstReleasedTimeSeconds;
      const nearSupport = Math.abs(positionM[2] - (SUPPORT_Z_M + BLOCK_HALF_Z_M)) < REST_Z_TOLERANCE_M;
      if (elapsedSinceRelease >= REST_MIN_SETTLE_SECONDS
        && nearSupport
        && this.inTarget
        && targetSupportContact
        && !gripperContact) {
        this.settleSeen = true;
      }
    }

    this.lastPositionM = [...positionM];
    this.lastSimulationTimeSeconds = simulationTimeSeconds;
    this.lastObservation = observation;
    return this.snapshot();
  }

  snapshot() {
    const finalPositionM = this.lastPositionM ? [...this.lastPositionM] : null;
    const success = Boolean(
      this.contactSeen
      && this.liftSeen
      && this.carrySeen
      && this.releaseSeen
      && this.settleSeen
      && this.inTarget
      && this.currentTargetSupportContact
      && !this.currentGripperContact
    );
    return Object.freeze({
      task: 'SO-101 Physical Block Transfer',
      physical: true,
      success,
      contactSeen: this.contactSeen,
      liftSeen: this.liftSeen,
      carrySeen: this.carrySeen,
      releaseSeen: this.releaseSeen,
      settleSeen: this.settleSeen,
      inTarget: this.inTarget,
      currentGripperContact: this.currentGripperContact,
      currentTargetSupportContact: this.currentTargetSupportContact,
      contactObservationCount: this.contactObservationCount,
      carriedContactObservationCount: this.carriedContactObservationCount,
      targetSupportContactObservationCount: this.targetSupportContactObservationCount,
      initialPositionM: this.initialPositionM ? [...this.initialPositionM] : null,
      finalPositionM,
      maxBlockZM: Number.isFinite(this.maxBlockZM) ? this.maxBlockZM : null,
      maxHorizontalTravelM: this.maxHorizontalTravelM,
      targetCenterXYM: [...this.goal.targetCenterXYM],
      targetHalfExtentsXYM: [...this.goal.targetHalfExtentsXYM],
      supportTopZM: SUPPORT_Z_M,
      blockHalfZM: BLOCK_HALF_Z_M,
      evidence: 'MuJoCo ground-truth body positions and named geometry contacts; settling requires post-release target-support contact and elapsed simulation time. No hardware-validation claim.',
    });
  }
}
