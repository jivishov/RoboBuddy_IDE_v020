import { PARAMETER_EVIDENCE } from './model-registry.js';

// Phase 5C source pins.
//
// P5C is a matched-environment integration, so it needs two pinned sources, not one.
// Neither is authority for everything:
//
//   * MICRODUCK_RUNTIME_SOURCE is the deployed controller/runtime authority. It supplies the
//     exact policy bytes, the 61-value observation contract, the joint order, the home pose,
//     the command encoding, the skill priority chain, the action scaling, the filters and the
//     50 Hz control cadence.
//   * MICRODUCK_RL_SOURCE is the physical/RL environment authority. It supplies the collision
//     set, the identified servo model, the physics timestep, the control decimation, the
//     reset pose and the ball prop - none of which exist in the deployed runtime source.
//
// The reconciliation between them is not assumed from names or joint counts. It is
// established below and asserted by tests/physics/microduck-phase5c-core.mjs.
export const MICRODUCK_RUNTIME_SOURCE = Object.freeze({
  repository: 'pollen-robotics/microduck',
  revision: '590b986bd8c0d50ae02cb3ea2f59c463b6828168',
  variant: 'alpha - the only shipped MicroDuck revision; every deployed policy is alpha_*',
  paths: Object.freeze([
    'kinematics/assets/alpha/robot_walk.xml',
    'duck-control/src/obs.rs',
    'duck-control/src/model.rs',
    'duck-control/src/policy.rs',
    'robotd/src/control.rs',
    'policies/README.md',
    'robotctl/assets/duck.bin',
  ]),
  license: 'Apache-2.0',
});

export const MICRODUCK_RL_SOURCE = Object.freeze({
  repository: 'pollen-robotics/microduck_rl',
  revision: '519142b1f5bf59fdfd44d06c205119e7fff8e3cb',
  revisionSubject: 'kick (2026-07-27)',
  paths: Object.freeze([
    'src/mjlab_microduck/robot/microduck/robot_walk.xml',
    'src/mjlab_microduck/robot/microduck/robot_allcollisions.xml',
    'src/mjlab_microduck/robot/microduck/joints_properties.xml',
    'src/mjlab_microduck/robot/microduck/scene.xml',
    'src/mjlab_microduck/robot/microduck/ball.xml',
    'src/mjlab_microduck/robot/microduck_constants.py',
    'src/mjlab_microduck/tasks/microduck_velstand_env_cfg.py',
    'src/mjlab_microduck/tasks/microduck_velocity_env_cfg.py',
    'src/mjlab_microduck/tasks/microduck_ball_kick_env_cfg.py',
    'scripts/infer_policy.py',
  ]),
  license: 'Apache-2.0 for code and MJCF; the STL mesh assets are CC BY-SA-NC and are NOT redistributed by this repository',
  selectionReason:
    'It is the newest revision whose robot_walk.xml is still the deployed model AND which already carries the source ball prop. '
    + 'The deployed-model window on this branch runs from 6594292 (2026-07-01, "update model (head_pitch in right direction)") to '
    + 'd649ce8 (2026-08-06); 1ee2e3c (2026-07-28, "update robot model with more accurate weights") re-weights the model and is '
    + 'therefore outside it. Current develop is outside it too, which is why this package does not track upstream head.',
});

// The historical training-run repository the deployed policy bytes were copied from.
// policies/README.md at the deployed pin names it and the exact commit for each file.
// It is recorded for provenance completeness; no parameter in this package comes from it,
// because the deployed repository already carries the exact bytes.
export const MICRODUCK_POLICY_ORIGIN_SOURCE = Object.freeze({
  repository: 'apirrone/microduck_runtime',
  revision: '5f3b314',
  rouladeRevision: '7e4ab6d',
  role: 'origin of the nine deployed ONNX policy files, vendored verbatim into the deployed runtime repository',
  status: 'not-inspected',
  reason: 'The deployed runtime repository at the pin carries the exact policy bytes this repository already serves, verified by SHA-256. Nothing further is needed from the origin repository, and it was not used as a parameter source.',
});

// The nine deployed policies and the role each one fills, from policies/README.md at the
// deployed pin. `net` is the label robotd's scheduler uses.
export const MICRODUCK_DEPLOYED_POLICIES = Object.freeze([
  Object.freeze({ id: 'walking', file: 'alpha_walking.onnx', net: 'walk', upstreamRun: 'BEST_alpha_walking_rough.onnx', role: 'walking / velstand', trainingTask: 'VelStand (velocity2 recipe + gated fall-recovery layer)' }),
  Object.freeze({ id: 'stand', file: 'alpha_stand.onnx', net: 'stand', upstreamRun: 'BEST_alpha_stand_body_control.onnx', role: 'standing + body-pose', trainingTask: 'StandUp / body-pose control' }),
  Object.freeze({ id: 'sitstand', file: 'alpha_sitstand.onnx', net: 'sit_stand', upstreamRun: 'BEST_alpha_sitstand.onnx', role: 'sit <-> stand (posture flag in the twist vx slot)', trainingTask: 'SitStand' }),
  Object.freeze({ id: 'ground_pick', file: 'alpha_ground_pick.onnx', net: 'ground_pick', upstreamRun: 'alpha_ground_pick.onnx', role: 'ground pick (phase command in the twist slots)', trainingTask: 'GroundPick' }),
  Object.freeze({ id: 'kick_left', file: 'ball_kick_left.onnx', net: 'kick_left', upstreamRun: 'ball_kick_left.onnx', role: 'left-leg kick', trainingTask: 'BallKick' }),
  Object.freeze({ id: 'kick_right', file: 'ball_kick_right.onnx', net: 'kick_right', upstreamRun: 'ball_kick_right.onnx', role: 'right-leg kick', trainingTask: 'BallKick' }),
  Object.freeze({ id: 'roulade', file: 'roulade.onnx', net: 'roulade', upstreamRun: 'roulade.onnx', role: 'forward roll', trainingTask: 'Mjlab-Roulade-MicroDuck' }),
  Object.freeze({ id: 'roller', file: 'roller.onnx', net: 'roller', upstreamRun: 'BEST_roller.onnx', role: 'roller-mode locomotion', trainingTask: 'VelocityRollers (roller-skate plant)' }),
  Object.freeze({ id: 'roller_crouch', file: 'roller_crouch.onnx', net: 'roller_crouch', upstreamRun: 'BEST_roller_crounch.onnx', role: 'roller-mode crouch', trainingTask: 'RollerCrouch (roller-skate plant)' }),
]);

// Exact SHA-256 of the nine deployed policy files. These are the bytes this repository
// serves from assets/microduck/policies/ and the bytes committed at the deployed pin;
// they were compared directly and are identical.
export const MICRODUCK_POLICY_SHA256 = Object.freeze({
  walking: 'e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c',
  stand: '1569268713e40deea795dd2922dba50d3621e15a872855408b6b1b125b1c094b',
  sitstand: 'c6c40e35e726eabd803d633e090d112994f469921152448367953fbaf9799bc8',
  ground_pick: 'ffbf5109982ff999b0ba53afe86b9ae731bbec679d67fb7f8ab4c52152c88872',
  kick_left: 'd6928284dccd3dd61e08bf2f760effa74309fbefd97b2b31afb2a60f526d196a',
  kick_right: '147a32c388c6b19111b3ac3b550a9a6dc8b8bf267118af4d8c3712522eedb5af',
  roulade: '3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b',
  roller: 'cf05651d2708a2f9364212e86b866c97a70ace8131c492500105e8f28bf99afd',
  roller_crouch: 'a1a084be240469c76ac9d3fa44d4792f16d4b1da60398b3ecd3cfc5e2244d990',
});

// The compatibility identity a physical policy is checked against before it may run.
// A mismatch in any field is a rejection, not a fidelity footnote.
export const MICRODUCK_COMPATIBILITY_SCHEMA_VERSION = 'microduck-p5c-1';

const row = (parameter, value, evidence, source, note) => Object.freeze({ parameter, value, evidence, source, note });

// Reconciliation table. Every physical and controller parameter the Phase 5C packages use
// appears here with its evidence class, its pinned source, and - where the two sources could
// have disagreed - the evidence that they do not.
export const MICRODUCK_RECONCILIATION = Object.freeze([
  row('robot variant', 'alpha', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/model.rs',
    'One variant. Every shipped policy is alpha_*; v1/v1.5/v1.6 are history in both sources.'),
  row('articulated hierarchy', '37 named bodies/joints/sites, 14 hinges, one free trunk', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck kinematics/assets/alpha/robot_walk.xml',
    'Compared entity-by-entity against microduck_rl robot_walk.xml at the selected revision: all 37 named entities present in both, '
    + 'with identical pos, quat, axis and range, and all 14 inertials identical. This is the evidence that the deployed runtime and the '
    + 'selected RL revision describe the same robot.'),
  row('total modelled mass', 0.744361, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'sum of the source inertials',
    'Compiled from the source inertials. Not a weighed robot; the CAD/inertial values are the source figures.'),
  row('home pose', '[0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0, 0, (mouth 0), 0, 0.0873, 0.4579, 0.0049, -0.4530]',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/model.rs DEFAULT_POSITION',
    'Identical to microduck_rl HOME_FRAME and to the STAND keyframe of the source scene. Both sources document it the same way: '
    + 'the STAND2 pose with the trunk ~5 mm forward so the CoM sits over the ankle axis. Observations are relative to it, so a '
    + 'discrepancy here would be a constant offset on fourteen observation slots.'),
  row('joint wire order', '15 slots, mouth at index 9', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-ipc-proto JOINT_NAMES, duck-control MOUTH_INDEX',
    'The physical model carries no mouth joint at all, matching the RL model; the wire order is preserved so the fourteen policy '
    + 'slots keep their published identity and the mouth slot cannot be filled by an off-by-one.'),
  row('observation layout', '61 = 3 gyro + 3 projected gravity + 14 (q - home) + 14 qdot + 14 previous raw action + 13 command',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/obs.rs', 'Single layout across every alpha policy; the runtime rejects any other width at load.'),
  row('command block', '[vx, vy, vyaw, neck_pitch, head_pitch, head_yaw, head_roll, 0, 0, body_z, body_roll, body_pitch, 0]',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/obs.rs',
    'Body x, y and yaw are unbound in training and are hardcoded zero. The body block is ordered z, roll, pitch.'),
  row('previous action semantics', 'raw policy output, before action scaling, shared across nets', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Controller::last_action', 'The policy was trained observing its own output, not the actuator command derived from it.'),
  row('observation history', 'none beyond the single previous action', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/obs.rs',
    'The 61-value contract carries exactly one previous action. No temporal stack exists in either source, so none is added.'),
  row('controller cadence', '50 Hz (20 ms)', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/main.rs, scripts/infer_policy.py',
    'The deployed loop runs at 50 Hz; the RL reference runner uses decimation 4 over a 5 ms physics step, which is the same 20 ms interval.'),
  row('physics timestep', 0.005, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'scripts/infer_policy.py (model.opt.timestep = 0.005)',
    'The RL reference runner sets it explicitly; four steps per controller tick gives the deployed 50 Hz.'),
  row('integrator / solver', 'Euler, 100 solver iterations, 50 line-search iterations, pyramidal cone', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl scene.xml compiled defaults', 'The source scene overrides none of them, so these are the values the reference runner actually executes.'),
  row('action scaling (walk)', 0.9, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/control.rs Tuning::action_scale',
    'Audited divergence: the training environment uses JointPositionAction scale 1.0, and the RL reference runner defaults to 1.0. '
    + 'The deployed daemon de-rates walking to 0.9. This package reproduces the deployed value because this repository pins the '
    + 'deployed runtime, and records 1.0 as the training value. Measured effect in the native reference: about 7% less distance over 6 s.'),
  row('action scaling (stand/kick/rise)', 1.0, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/control.rs standing_action_scale',
    'Standing tuning also applies to a kick window and to the sitstand rise, because both present an all-zero effective command.'),
  row('action scaling (ground pick, roulade)', 1.0, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/control.rs SkillTuning', null),
  row('target low-pass filters', 'head joints alpha 0.5, other joints alpha 0.7', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Tuning::head_lowpass / legs_lowpass',
    'Audited divergence: the deployed source states the alpha policies are trained with these values, while the RL reference runner '
    + 'applies no filter and the training tree reverted its low-pass experiment. This package reproduces the deployed filters and '
    + 'reports both. Measured effect in the native reference: under 4% of walking distance either way.'),
  row('standing threshold', 0.05, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/policy.rs DEFAULT_STANDING_THRESHOLD',
    'Velocity-command magnitude below which the standing network takes over.'),
  row('skill priority chain', 'roulade > kick > ground pick > sit/rise > stand-by-magnitude > walk', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Controller::step', null),
  row('firmware position gain', 200, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/control.rs Tuning::gain, microduck_constants.py kp_fw',
    'The deployed daemon and the RL actuator configuration agree on the XL330 firmware position gain, which is what makes the identified '
    + 'servo model below the matching one.'),
  row('servo model', 'MuJoCo position actuator kp 0.55 N m/rad, kv 0, force range +/-0.96 N m, control range +/-10 rad',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl joints_properties.xml class "chosen_actuator"',
    'An identified XL330 model at firmware kp 200, not a generic PD guess. Audited divergence: PPO training used the BAM M6 voltage-domain '
    + 'actuator with 3-6 tick action delay and voltage/friction randomisation; this identified position model is the one the upstream CPU '
    + 'reference runner executes, and it is what a browser MuJoCo build can run. The BAM voltage model, its delay buffer and its domain '
    + 'randomisation are not reproduced, and that is the largest declared actuator-fidelity gap in this package.'),
  row('joint damping / frictionloss / armature', [0.053, 0.0048, 0.0018], PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl joints_properties.xml class "chosen_actuator"', 'Identified alongside the servo gain, from the same test-bench fit.'),
  row('collision set', '15 colliders: 2 soles, 2 hips, 2 thighs, 2 shanks, 2 trunk shells, battery, power support (self-collision only), 3 head shells',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl robot_allcollisions.xml',
    'The identity, parent body, placement and contact mask of every collider are source-derived. Their shape is not: see the next row.'),
  row('collider primitive fit', 'repository-authored boxes fitted to the compiled source mesh envelopes', PARAMETER_EVIDENCE.ESTIMATED,
    'measured from the compiled upstream model',
    'The upstream colliders are CC BY-SA-NC meshes that this repository does not redistribute. Each is replaced by a box fitted to the '
    + 'source mesh envelope in the same body frame. Most parts are boxier than they are round (over half of their vertices lie outside the '
    + 'inscribed ellipsoid of their own bounding box), so a box is the closer primitive.'),
  row('sole contact face', 'box 45.6 x 34.0 mm, 6 mm thick, rolled 4.75 deg about the ankle x axis', PARAMETER_EVIDENCE.ESTIMATED,
    'measured from the compiled upstream sole mesh',
    'The only load-bearing walking contact, so it is fitted to the measured sole contact face - the vertices within 1 mm of the sole plane - '
    + 'rather than to the mesh bounding box (54.0 x 41.2 mm). Fitting the bounding box would have silently enlarged the support polygon by 45%. '
    + 'The left and right fits agree to 6 micrometres.'),
  row('contact parameters', 'condim 3, friction (1.0, 0.005, 0.0001), solref (0.02, 1), default solimp', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl scene.xml as compiled by the upstream CPU reference runner',
    'Audited divergence: PPO training additionally applies mjlab FULL_COLLISION, which sets condim 1 on the non-foot colliders and gives the '
    + 'feet contact priority. The reference runner - the path that actually executes a deployed ONNX policy - compiles the raw scene, so that '
    + 'is what this package reproduces.'),
  row('floor', 'infinite plane at z = 0, same friction and solref as the robot colliders', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl scene.xml', 'A declared flat indoor floor. Not a measured surface.'),
  row('reduced-traction surface', 0.02, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored adverse-condition fixture',
    'A deliberately degraded surface for the traction gate, matching the value the LeKiwi package already uses for the same purpose. Not a modelled real material.'),
  row('reset state', 'trunk at z = 0.12, identity orientation, joints at the home pose, actuator targets at the home pose',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl scene.xml STAND keyframe',
    'The source keyframe spawns the robot 2.8 mm clear of the floor and lets it settle; this package keeps that exactly.'),
  row('ball radius / mass / inertia', [0.035, 0.015, 1.225e-5], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl ball.xml',
    '70 mm diameter, 15 g hollow plastic ball; the inertia is the thin-hollow-sphere value the source states.'),
  row('ball friction', [0.5, 0.005, 0.0001], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl ball.xml',
    'Low rolling resistance so a kicked ball rolls. There is no separate rolling-resistance rule anywhere in the physical path.'),
  row('ball placement', 'x 0.09, y -0.042 in the robot yaw frame', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl microduck_ball_kick_env_cfg.py BALL_OFFSET', 'The nominal kick-task placement, in front of the right foot. Training added +/-15 mm of placement noise, which this package does not apply by default.'),
  row('gyro source', 'MuJoCo gyro sensor at the source-named imu site', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl sensors.xml, scripts/infer_policy.py', 'The imu site carries identity rotation relative to the trunk, so this is the trunk-frame angular velocity the deployed controller reads.'),
  row('projected gravity source', 'world -Z rotated into the trunk body frame', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'scripts/infer_policy.py get_projected_gravity', 'Taken from the trunk body quaternion, not from a simulated accelerometer.'),
  row('observation noise and delay', 'none applied', PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored',
    'Training randomised IMU delay, encoder bias and observation noise. The physical workspace publishes clean simulator ground truth, so '
    + 'measured robustness margins are not claimed.'),
  row('hardware alignment', 'none', PARAMETER_EVIDENCE.CALIBRATION_REQUIRED, '-',
    'No measurement of an assembled MicroDuck was used anywhere in Phase 5C. Walking speed, traction, stability margin, kick distance, '
    + 'recovery probability and actuator dynamics are all unvalidated against hardware.'),
]);

export function reconciliationByEvidence(evidence) {
  return MICRODUCK_RECONCILIATION.filter((item) => item.evidence === evidence);
}

export function assertReconciliationCoverage() {
  const seen = new Set(MICRODUCK_RECONCILIATION.map((item) => item.parameter));
  const required = [
    'robot variant', 'articulated hierarchy', 'home pose', 'joint wire order', 'observation layout',
    'command block', 'previous action semantics', 'observation history', 'controller cadence',
    'physics timestep', 'action scaling (walk)', 'target low-pass filters', 'servo model',
    'joint damping / frictionloss / armature', 'collision set', 'collider primitive fit',
    'sole contact face', 'contact parameters', 'reset state', 'hardware alignment',
  ];
  const missing = required.filter((name) => !seen.has(name));
  if (missing.length) throw new Error(`MicroDuck reconciliation table is missing: ${missing.join(', ')}`);
  for (const item of MICRODUCK_RECONCILIATION) {
    if (!Object.values(PARAMETER_EVIDENCE).includes(item.evidence)) {
      throw new Error(`MicroDuck reconciliation row ${item.parameter} has unknown evidence ${item.evidence}`);
    }
  }
  return MICRODUCK_RECONCILIATION.length;
}
