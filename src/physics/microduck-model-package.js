import { PARAMETER_EVIDENCE, registerModelPackage } from './model-registry.js';
import { MICRODUCK_RL_SOURCE, MICRODUCK_RUNTIME_SOURCE } from './microduck-source-audit.js';
import {
  MICRODUCK_CONTROL_DECIMATION,
  MICRODUCK_HOME_POSITION_RAD,
  MICRODUCK_PHYSICS_TIMESTEP_SECONDS,
  MICRODUCK_POLICY_JOINT_ORDER,
} from './microduck-controller.js';

export const MICRODUCK_ROBOT_ID = 'microduck_alpha_biped';
export const MICRODUCK_JOINT_CONTROLLER = 'microduck_joint_position';
export const MICRODUCK_FLOOR_ID = 'microduck_floor';
export const MICRODUCK_TRUNK_BODY = 'trunk_base';
export const MICRODUCK_BALL_BODY = 'microduck_ball';
export const MICRODUCK_FOOT_GEOMS = Object.freeze(['left_foot_collision', 'right_foot_collision']);
export const MICRODUCK_BALL_GEOM = 'microduck_ball_geom';

// microduck_rl joints_properties.xml, class "chosen_actuator" - an identified XL330 model at
// the firmware position gain the deployed daemon ships, not a generic PD guess.
export const MICRODUCK_SERVO_KP_NM_RAD = 0.55;
export const MICRODUCK_SERVO_KV = 0;
export const MICRODUCK_SERVO_FORCE_NM = 0.96;
export const MICRODUCK_SERVO_CTRL_RAD = 10;

// The firmware gain the identified stiffness corresponds to, and the mapping between them.
//
// This is not inferred. microduck_rl's `chosen_actuator` class - the one the robot's joints
// actually use - carries `<!-- 200 kp -->` directly above `kp="0.55"`, and its commented-out
// alternative carries `<!-- 125 kp -->` above `kp="0.35"`. Those two points are proportional
// to within 2% (0.00275 vs 0.00280 per firmware unit), so a firmware gain maps onto the
// identified stiffness by ratio, and the mapping is exact at the robot's own gain of 200.
//
// The same pair settles a question that guessing would get wrong: the torque limit does NOT
// move with the gain. The 125 kp variant keeps `forcerange="-0.96 0.96"`.
//
// This matters because the deployed daemon does not hold one gain. duck-control/src/bus.rs
// writes a position-P-gain register on every servo every tick, and robotd/src/control.rs
// schedules it: standing, kicks and the sit/rise cycle run at 0.8 of the running gain.
export const MICRODUCK_NOMINAL_FIRMWARE_GAIN = 200;

/**
 * The identified MuJoCo stiffness for a deployed firmware gain.
 * Gain 0 is a true torque-off: the actuator produces no force at all.
 */
export function microDuckKpForFirmwareGain(gain) {
  const value = Number(gain);
  if (!Number.isFinite(value) || value < 0) throw new TypeError('Firmware gain must be a finite, non-negative number');
  return MICRODUCK_SERVO_KP_NM_RAD * value / MICRODUCK_NOMINAL_FIRMWARE_GAIN;
}
export const MICRODUCK_JOINT_DAMPING = 0.053;
export const MICRODUCK_JOINT_FRICTIONLOSS = 0.0048;
export const MICRODUCK_JOINT_ARMATURE = 0.0018;

// microduck_rl ball.xml.
export const MICRODUCK_BALL_RADIUS_M = 0.035;
export const MICRODUCK_BALL_MASS_KG = 0.015;
// microduck_ball_kick_env_cfg.py BALL_OFFSET, in the robot yaw frame.
export const MICRODUCK_BALL_OFFSET_M = Object.freeze([0.09, -0.042]);

export const MICRODUCK_NOMINAL_FLOOR_FRICTION = 1.0;
export const MICRODUCK_LOW_TRACTION_FRICTION = 0.02;

const RUNTIME_URL = `https://github.com/${MICRODUCK_RUNTIME_SOURCE.repository}/blob/${MICRODUCK_RUNTIME_SOURCE.revision}/kinematics/assets/alpha/robot_walk.xml`;
const RL_URL = `https://github.com/${MICRODUCK_RL_SOURCE.repository}/blob/${MICRODUCK_RL_SOURCE.revision}/src/mjlab_microduck/robot/microduck`;

// Exact joint ranges, read from the pinned Apache-2.0 source model and asserted against the
// compiled model by the worker.
const JOINT_RANGES = Object.freeze({
  left_hip_yaw: [-0.4363323129985824, 0.5235987755982988],
  left_hip_roll: [-0.3839724354387516, 0.38397243543875337],
  left_hip_pitch: [-1.5707963267949037, 1.5707963267948895],
  left_knee: [-1.570796326794901, 1.5707963267948921],
  left_ankle: [-1.5707963267949019, 1.5707963267948912],
  neck_pitch: [-1.5707963267948966, 1.0471975511965976],
  head_pitch: [-1.5707963267948966, 1.5707963267948966],
  head_yaw: [-2.967059728390364, 2.967059728390357],
  head_roll: [-0.4363323129986037, 0.43633231299856107],
  right_hip_yaw: [-0.5235987755982988, 0.4363323129985824],
  right_hip_roll: [-0.3839724354387525, 0.3839724354387525],
  right_hip_pitch: [-1.5707963267949, 1.570796326794893],
  right_knee: [-1.570796326794901, 1.5707963267948921],
  right_ankle: [-1.5707963267949028, 1.5707963267948903],
});

const joints = () => MICRODUCK_POLICY_JOINT_ORDER.map((id) => ({
  id,
  rangeRad: [...JOINT_RANGES[id]],
  axis: [0, 0, 1],
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
}));

const actuators = () => MICRODUCK_POLICY_JOINT_ORDER.map((id) => ({
  id: `act_${id}`,
  jointId: id,
  controllerId: MICRODUCK_JOINT_CONTROLLER,
  command: 'position-rad',
  controlRangeRad: [-MICRODUCK_SERVO_CTRL_RAD, MICRODUCK_SERVO_CTRL_RAD],
  forceRangeNm: [-MICRODUCK_SERVO_FORCE_NM, MICRODUCK_SERVO_FORCE_NM],
  evidence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
}));

const initialJointPositionsRad = () => Object.fromEntries(
  MICRODUCK_POLICY_JOINT_ORDER.map((id, slot) => [id, MICRODUCK_HOME_POSITION_RAD[slot]]),
);

const ROBOT_BODIES = Object.freeze([
  { id: MICRODUCK_TRUNK_BODY, freeJointId: 'trunk_base_freejoint' },
  { id: 'ankle_left' }, { id: 'ankle_right' },
  { id: 'bottom_head_shell' },
  { id: 'left_upper_leg' }, { id: 'right_upper_leg' },
]);

const BASE_EVIDENCE = Object.freeze({
  articulatedHierarchy: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  jointAxesAndRanges: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  linkInertials: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  namedSitesAndFrames: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  homePose: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  actuatorModel: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  jointDampingFrictionArmature: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  physicsTimestepAndCadence: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  collisionSetIdentityAndPlacement: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  contactParameters: PARAMETER_EVIDENCE.SOURCE_DERIVED,
  colliderPrimitiveShape: PARAMETER_EVIDENCE.ESTIMATED,
  actuatorVoltageAndDelayModel: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
  hardwareAlignment: PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
});

const BASE_LIMITATIONS = Object.freeze([
  'The articulated hierarchy, body transforms, joint axes, joint ranges, link inertials and named sites are read verbatim from the Apache-2.0 '
  + `${MICRODUCK_RUNTIME_SOURCE.repository}@${MICRODUCK_RUNTIME_SOURCE.revision.slice(0, 7)} kinematics/assets/alpha/robot_walk.xml that this repository already serves. `
  + `The collision set, the servo model and the physics cadence come from ${MICRODUCK_RL_SOURCE.repository}@${MICRODUCK_RL_SOURCE.revision.slice(0, 7)}, whose robot model is structurally identical to the deployed one.`,
  'The upstream colliders are CC BY-SA-NC mesh assets that this repository does not redistribute. Every collider here is a repository-authored box '
  + 'fitted to the corresponding source mesh envelope. The two soles are fitted to the measured sole contact face (45.6 x 34.0 mm), not to the mesh '
  + 'bounding box (54.0 x 41.2 mm), so the support polygon is not silently enlarged.',
  'The servo is the identified microduck_rl "chosen_actuator" position model - kp 0.55 N m/rad at firmware gain 200, force range +/-0.96 N m, with '
  + 'joint damping 0.053, friction loss 0.0048 and armature 0.0018. The BAM M6 voltage-domain actuator the policies were trained against, including '
  + 'its 3-6 tick action delay and its voltage/friction randomisation, is NOT reproduced. That is the largest declared actuator-fidelity gap here.',
  'Contact stiffness, friction and the floor are the declared parameters of the upstream reference scene on a flat indoor plane. Carpet, slopes, '
  + 'thresholds, rough terrain, foot wear, battery droop and servo thermal behaviour are outside the modelled scope.',
  'The trunk is a free MuJoCo body. Nothing in this package writes root pose, root velocity, joint state or object state outside explicit, logged '
  + 'setup and reset. There is no weld, equality constraint, attachment, snap, teleport, boundary clamp or task-success overwrite anywhere in it.',
  'No hardware comparison exists. Walking speed, traction, stability margin, kick distance, recovery probability and actuator dynamics are all '
  + 'unvalidated against an assembled MicroDuck.',
]);

function basePackage({ id, modelId, asset, sha256, variant, floorFriction, limitations = [] }) {
  return {
    id,
    modelId,
    asset,
    sha256,
    robotId: MICRODUCK_ROBOT_ID,
    source: {
      url: RUNTIME_URL,
      revision: MICRODUCK_RUNTIME_SOURCE.revision,
      variant,
      physicalEnvironmentUrl: RL_URL,
      physicalEnvironmentRevision: MICRODUCK_RL_SOURCE.revision,
    },
    license: 'Apache-2.0 for the MicroDuck-derived hierarchy and reconciled physical parameters; the repository-authored colliders and floor are MIT. No CC BY-SA-NC mesh asset is redistributed.',
    physics: { timestepSeconds: MICRODUCK_PHYSICS_TIMESTEP_SECONDS, integrator: 'Euler', iterations: 100, lsIterations: 50 },
    controllers: [MICRODUCK_JOINT_CONTROLLER],
    joints: joints(),
    actuators: actuators(),
    bodies: [...ROBOT_BODIES],
    initialJointPositionsRad: initialJointPositionsRad(),
    sceneConstraints: { fixtures: [MICRODUCK_FLOOR_ID], objects: [] },
    controlDecimation: MICRODUCK_CONTROL_DECIMATION,
    floorFriction,
    evidence: BASE_EVIDENCE,
    limitations: [...BASE_LIMITATIONS, ...limitations],
  };
}

export const MICRODUCK_WALK_PACKAGE = registerModelPackage(basePackage({
  id: 'microduck-walk-519142b-v1',
  modelId: 'robobuddy-microduck-walk-v1',
  asset: 'models/microduck/walk.xml',
  sha256: '13149fa946290b9f4a0ac7029e511fed8cb2e59a6e507098a758cc7e190b207c',
  variant: 'MicroDuck alpha free-base biped with the reconciled ground-contact collision set on a flat indoor floor',
  floorFriction: MICRODUCK_NOMINAL_FLOOR_FRICTION,
}));

export const MICRODUCK_LOW_TRACTION_PACKAGE = registerModelPackage(basePackage({
  id: 'microduck-walk-lowtraction-519142b-v1',
  modelId: 'robobuddy-microduck-walk-lowtraction-v1',
  asset: 'models/microduck/walk_lowtraction.xml',
  sha256: '15ad5285fdcde992e497f749f1495a56261e922806ee492fa08dbdc84a7c6883',
  variant: 'MicroDuck alpha free-base biped on a deliberately reduced-traction surface (sliding friction 0.02)',
  floorFriction: MICRODUCK_LOW_TRACTION_FRICTION,
  limitations: ['Adverse-condition fixture: sole and floor sliding friction are reduced to 0.02. This is a declared degraded surface, not a modelled real material.'],
}));

export const MICRODUCK_KICK_PACKAGE = registerModelPackage((() => {
  const base = basePackage({
    id: 'microduck-kick-519142b-v1',
    modelId: 'robobuddy-microduck-kick-v1',
    asset: 'models/microduck/kick.xml',
    sha256: 'c857f967d9254017cdd7b06ad65ac89c53f060cfae682396ad4a0c434c2f4c62',
    variant: 'MicroDuck alpha free-base biped plus the source 70 mm / 15 g ball prop at the source kick-task placement',
    floorFriction: MICRODUCK_NOMINAL_FLOOR_FRICTION,
  });
  return {
    ...base,
    bodies: [...base.bodies, { id: MICRODUCK_BALL_BODY, freeJointId: 'microduck_ball_free' }],
    sceneConstraints: { fixtures: [MICRODUCK_FLOOR_ID], objects: [MICRODUCK_BALL_BODY] },
    evidence: { ...base.evidence, ballMassRadiusInertia: PARAMETER_EVIDENCE.SOURCE_DERIVED, ballFriction: PARAMETER_EVIDENCE.SOURCE_DERIVED, ballPlacement: PARAMETER_EVIDENCE.SOURCE_DERIVED },
    limitations: [
      ...base.limitations,
      'The ball is the source prop from microduck_rl ball.xml: 70 mm diameter, 15 g, thin-hollow-sphere inertia 1.225e-5 kg m^2, friction (0.5, 0.005, 0.0001). '
      + 'It is placed at the source kick-task offset (0.09, -0.042) m in the robot yaw frame. Training additionally randomised that placement by +/-15 mm; this package does not.',
      'Ball motion comes only from MuJoCo contact. There is no kick impulse, no ball velocity overwrite, no rolling-resistance rule and no synthetic contact inference anywhere in this package.',
    ],
  };
})());

export const MICRODUCK_MODEL_PACKAGES = Object.freeze([
  MICRODUCK_WALK_PACKAGE,
  MICRODUCK_LOW_TRACTION_PACKAGE,
  MICRODUCK_KICK_PACKAGE,
]);
