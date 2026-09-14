export * from './openarm-lab-builder-simulator-base.js';
import { OpenArmLabBuilderSimulator as BaseSimulator } from './openarm-lab-builder-simulator-base.js';
import { OPENARM_LAB_OBSERVATION_PROFILES, OpenArmSyntheticEstimator } from './openarm-lab-observation-profile.js';
import { createLabProject } from './openarm-lab-builder.js';

const clone = value => structuredClone(value);
const distance3 = (a, b) => Math.hypot(...a.map((value, index) => value - b[index]));

export class OpenArmLabBuilderSimulator extends BaseSimulator {
  constructor(...args) {
    super(...args);
    this.observationProfileId = 'simulator_ground_truth';
    this.estimatorSeed = 1;
    this.syntheticEstimator = new OpenArmSyntheticEstimator({ seed: this.estimatorSeed });
    this.lastPlanBinding = null;
  }
  async setScenario(...args) {
    const result = await super.setScenario(...args);
    this.lastPlanBinding = null;
    this.syntheticEstimator.reset(this.estimatorSeed);
    if (this.lastObservation) this.syntheticEstimator.push(this.lastObservation);
    return result;
  }
  async reset(...args) {
    const result = await super.reset(...args);
    this.lastPlanBinding = null;
    this.syntheticEstimator.reset(this.estimatorSeed);
    if (this.lastObservation) this.syntheticEstimator.push(this.lastObservation);
    return result;
  }
  async advanceTime(...args) {
    const observation = await super.advanceTime(...args);
    if (observation) this.syntheticEstimator.push(observation);
    return observation;
  }
  async applyPhysicalTargets(...args) {
    const result = await super.applyPhysicalTargets(...args);
    if (result?.observation) this.syntheticEstimator.push(result.observation);
    return result;
  }
  async applyToolTarget(...args) {
    const result = await super.applyToolTarget(...args);
    if (result?.observation) this.syntheticEstimator.push(result.observation);
    return result;
  }
  setObservationProfile(profileId, { seed = this.estimatorSeed } = {}) {
    if (!OPENARM_LAB_OBSERVATION_PROFILES[profileId]) throw new TypeError(`Unknown observation profile ${profileId}`);
    if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) throw new RangeError('Estimator seed must be a uint32');
    this.observationProfileId = profileId;
    this.estimatorSeed = seed >>> 0;
    this.syntheticEstimator.reset(this.estimatorSeed);
    if (this.lastObservation) this.syntheticEstimator.push(this.lastObservation);
    return { profile: clone(OPENARM_LAB_OBSERVATION_PROFILES[profileId]), seed: this.estimatorSeed };
  }
  getSensorObservation() {
    if (!this.lastObservation) return { valid: false, reason: 'no_observation', profileId: this.observationProfileId, hardwareValidated: false };
    if (this.observationProfileId === 'simulator_ground_truth') return {
      valid: true,
      profileId: 'simulator_ground_truth',
      observation: clone(this.lastObservation),
      source: 'MuJoCo authoritative state exposed only by the development profile',
      hardwareValidated: false,
      cameraPerception: false,
    };
    this.syntheticEstimator.push(this.lastObservation);
    return this.syntheticEstimator.observe(this.lastObservation.simulationTimeSeconds);
  }
  getWorkcellState() {
    const state = super.getWorkcellState();
    return {
      ...state,
      observationMode: this.observationProfileId,
      observationProfiles: clone(OPENARM_LAB_OBSERVATION_PROFILES),
      estimatorSeed: this.estimatorSeed,
      generatedPlanBinding: clone(this.lastPlanBinding),
      preHardwarePackageStatus: 'partial: synthetic estimator implemented; controller-side observation enforcement and perturbation rerun package remain open',
    };
  }
  getTelemetry() {
    const observation = this.lastObservation;
    if (!observation) return {};
    const result = { simulation_time_s: observation.simulationTimeSeconds };
    for (const [id, joint] of Object.entries(observation.joints || {})) result[`${id}_rad`] = joint.positionRad;
    for (const record of observation.openarm?.equipment || []) {
      const position = observation.bodies?.[record.bodyId]?.positionM;
      if (!Array.isArray(position)) continue;
      position.forEach((value, index) => { result[`${record.id}_${'xyz'[index]}_m`] = value; });
    }
    return result;
  }
  planLabTransfer(...args) {
    const result = super.planLabTransfer(...args);
    if (!result?.supported || !result.program) {
      this.lastPlanBinding = null;
      return result;
    }
    const authority = this.getPhysicalAuthorityToken();
    const relevantBodyPositionsM = {};
    for (const asset of this.sceneSpec?.assets || []) {
      if (!asset.dynamic && ![this.taskSpec?.object_id, this.taskSpec?.receiver_id].includes(asset.id)) continue;
      const position = this.lastObservation?.bodies?.[`lab_${asset.id}`]?.positionM;
      if (Array.isArray(position)) relevantBodyPositionsM[asset.id] = [...position];
    }
    this.lastPlanBinding = {
      sessionId: authority?.sessionId || null,
      epoch: authority?.epoch ?? null,
      sceneRevision: authority?.sceneRevision || null,
      simulationTimeSeconds: this.lastObservation?.simulationTimeSeconds ?? null,
      relevantBodyPositionsM,
      positionValidityToleranceM: 0.01,
      programKey: JSON.stringify(result.program),
    };
    return { ...result, planner: { ...result.planner, stateBinding: clone(this.lastPlanBinding) } };
  }
  setLabProgramSpec(program) {
    const key = JSON.stringify(program);
    if (this.lastPlanBinding && key === this.lastPlanBinding.programKey) {
      const authority = this.getPhysicalAuthorityToken();
      if (!authority || authority.sessionId !== this.lastPlanBinding.sessionId || authority.epoch !== this.lastPlanBinding.epoch || authority.sceneRevision !== this.lastPlanBinding.sceneRevision) {
        throw new Error('Generated transfer plan is stale because the authoritative simulation session, epoch, or scene revision changed; replan from the current state.');
      }
      for (const [assetId, plannedPosition] of Object.entries(this.lastPlanBinding.relevantBodyPositionsM || {})) {
        const current = this.lastObservation?.bodies?.[`lab_${assetId}`]?.positionM;
        if (!Array.isArray(current) || distance3(current, plannedPosition) > this.lastPlanBinding.positionValidityToleranceM) {
          throw new Error(`Generated transfer plan is stale because ${assetId} moved beyond the declared 10 mm validity tolerance; re-observe and replan.`);
        }
      }
    } else {
      // A deliberately edited program is a new instruction artifact rather than the old planner output.
      this.lastPlanBinding = null;
    }
    return super.setLabProgramSpec(program);
  }
  exportLabProject() {
    return createLabProject({
      sceneSpec: this.sceneSpec,
      taskSpec: this.taskSpec,
      program: this.programSpec,
      executionProfile: {
        schema_version: 'robobuddy.lab.execution.v1',
        id: this.observationProfileId,
        observation_mode: this.observationProfileId,
        estimator_seed: this.estimatorSeed,
        controller: 'openarm_v2_position',
        controller_model_source: 'existing OpenArm simulation control profile',
        plant_parameter_source: 'active MuJoCo model package; not silently read as controller calibration',
        hardware_validated: false,
      },
    });
  }
}
