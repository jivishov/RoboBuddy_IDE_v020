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
    this.programValidation = null;
  }
  async setScenario(...args) {
    const result = await super.setScenario(...args);
    this.lastPlanBinding = null;
    this.programValidation = null;
    this.syntheticEstimator.reset(this.estimatorSeed);
    if (this.lastObservation) this.syntheticEstimator.push(this.lastObservation);
    return result;
  }
  async reset(...args) {
    const result = await super.reset(...args);
    this.lastPlanBinding = null;
    this.programValidation = null;
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
      programValidation: clone(this.programValidation),
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
      this.programValidation = null;
      return result;
    }

    // Add feedback where it materially constrains the physical transfer. The independent
    // evaluator remains authoritative; these waits only keep the generated controller from
    // blindly advancing after missing receiver support or a failed release.
    const program = clone(result.program);
    const equipment = this.lastObservation?.openarm?.equipment || [];
    const objectRecord = equipment.find(record => record.id === this.taskSpec?.object_id);
    const receiverRecord = equipment.find(record => record.id === this.taskSpec?.receiver_id);
    const objectBodyId = objectRecord?.bodyId || `lab_${this.taskSpec?.object_id}`;
    const supportGeometryId = receiverRecord?.affordances?.supportGeometryId || `lab_${this.taskSpec?.receiver_id}_base`;
    const lowerIndex = program.segments.findIndex(segment => /lower into receiver/i.test(segment.label || ''));
    const releaseIndex = program.segments.findIndex(segment => /open and release/i.test(segment.label || ''));
    if (lowerIndex >= 0) {
      program.segments[lowerIndex].wait_for = {
        type: 'supported', object_id: objectBodyId, support_geom: supportGeometryId,
        timeout_seconds: 2, dwell_seconds: .06,
      };
    }
    if (releaseIndex >= 0) {
      const release = program.segments[releaseIndex];
      delete release.wait_for;
      release.label = 'Open gripper while receiver supports object';
      release.duration_seconds = Math.max(1.5, Number(release.duration_seconds) || 0);
      const lowerTool = lowerIndex >= 0 ? program.segments[lowerIndex].tool : null;
      if (lowerTool?.position_m) {
        const clearance = clone(lowerTool);
        clearance.position_m[2] += .025;
        program.segments.splice(releaseIndex + 1, 0,
          {
            label: 'Clear released object with open gripper', tool: clearance, duration_seconds: 1.5,
            wait_for: { type: 'released', side: this.taskSpec.side, object_id: objectBodyId, timeout_seconds: 2, dwell_seconds: .08 },
          },
          {
            label: 'Allow supported object to settle',
            duration_seconds: Math.max(.35, Number(this.taskSpec?.tolerances?.settle_dwell_s || .2) + .15),
          },
        );
      }
    }
    this.programSpec = clone(program);

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
      programKey: JSON.stringify(program),
    };
    this.programValidation = {
      origin: 'lab_transfer_planner',
      stateBound: true,
      taskSpaceClearanceValidated: true,
      sampledRobotLinkClearanceValidated: false,
      continuousCollisionGuarantee: false,
    };
    return { ...result, program: clone(program), planner: { ...result.planner, stateBinding: clone(this.lastPlanBinding), feedback: 'receiver support before release; sustained release after 25 mm clearance; explicit settle dwell', sampledRobotLinkClearanceValidated: false } };
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
      this.programValidation = { ...(this.programValidation || {}), origin: 'lab_transfer_planner', stateBound: true };
    } else {
      // A deliberately edited/external program is permitted as a new bounded instruction artifact,
      // but it must not inherit the planner's clearance or freshness claims.
      this.lastPlanBinding = null;
      this.programValidation = { origin: 'edited_or_external', stateBound: false, taskSpaceClearanceValidated: false, sampledRobotLinkClearanceValidated: false, continuousCollisionGuarantee: false };
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
        program_validation: clone(this.programValidation),
        hardware_validated: false,
      },
    });
  }
}
