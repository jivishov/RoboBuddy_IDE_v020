import { OPENARM_CONTROL_PROFILE } from './openarm-servo.js';

export const OPENARM_LAB_EXECUTION_PROFILE_VERSION = 'robobuddy.openarm.lab-execution.v1';

export const OPENARM_LAB_EXECUTION_PROFILES = Object.freeze({
  reference_simulation_v1: Object.freeze({
    schema_version: OPENARM_LAB_EXECUTION_PROFILE_VERSION,
    id: 'reference_simulation_v1',
    label: 'Reference browser simulation',
    observation_profile: 'simulator_ground_truth',
    command_semantics: 'bounded joint position references or Cartesian pinch targets converted to bounded joint position references',
    external_command_period_s: 0,
    command_transport_delay_s: 0,
    internal_controller: OPENARM_CONTROL_PROFILE.id,
    internal_controller_update_period_s: .001,
    effort_limits: 'source actuator force ranges with the repository-designed 1.2 N*m gripper operating cap',
    plant_dynamics: 'source-derived OpenArm V2 MuJoCo plant plus explicitly labelled estimated dry-workcell contact/material parameters',
    evidence: 'reference simulator configuration; not hardware firmware or installed-hardware calibration',
    hardware_validated: false,
  }),
  assumed_hardware_interface_v1: Object.freeze({
    schema_version: OPENARM_LAB_EXECUTION_PROFILE_VERSION,
    id: 'assumed_hardware_interface_v1',
    label: 'Declared pre-hardware interface assumption',
    observation_profile: 'synthetic_estimator_v1',
    command_semantics: '50 Hz bounded position-command interface; Cartesian requests are resolved to joint references before command issue',
    external_command_period_s: .02,
    command_transport_delay_s: .02,
    internal_controller: OPENARM_CONTROL_PROFILE.id,
    internal_controller_update_period_s: .001,
    effort_limits: 'same bounded simulation actuator ranges as the reference plant; no undocumented hardware motor-limit claim',
    plant_dynamics: 'same source-derived/estimated MuJoCo plant used for pre-hardware sensitivity only; not a hardware dynamics replica',
    evidence: 'assumed interface used for delay/noise sensitivity testing because installed camera/encoder/tactile/firmware details are not established in this project',
    hardware_validated: false,
  }),
});

export function requireOpenArmLabExecutionProfile(profileId) {
  const profile = OPENARM_LAB_EXECUTION_PROFILES[profileId];
  if (!profile) throw new TypeError(`Unknown Lab Builder execution profile ${profileId}`);
  return profile;
}
