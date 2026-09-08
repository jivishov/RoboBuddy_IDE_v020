# SO-101 Phase 2A model provenance

This package is a controlled articulated-plant validation model for RoboBuddy IDE Physics Preview Phase 2A. It is not a hardware-calibrated digital twin, it is not dynamically identical to the complete pinned MuJoCo Menagerie package, and it is not yet the production SO-101 workspace.

## Authoritative source

- Upstream package: Google DeepMind MuJoCo Menagerie, `robotstudio_so101`.
- Pinned Menagerie revision: `8161bba264d7fa7c99ca301e91e7fb44737676ad`.
- Source MJCF: `robotstudio_so101/so101.xml`.
- Menagerie provenance: derived from The Robot Studio SO-101 `so101_new_calib.xml`; the Menagerie README records copied source revision `aec17bbc256d1a7342d53aaa4950595d4c30b40d`.
- Source variant: The Robot Studio SO-101 follower arm using STS3215-class servos and the single moving-jaw gripper described by the pinned Menagerie MJCF.
- RoboBuddy Phase 2A adaptation: the six-actuated-joint chain from that source, with a deliberately reduced self-contained geometry set and without the source `camera_mount` child.
- License: Apache License 2.0, as declared by the pinned Menagerie SO-101 package.

Source URLs:

- https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101
- https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/so101.xml
- https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/LICENSE

## Upstream source asset inventory

The pinned Menagerie MJCF references these visual/source meshes:

- `waveshare_mounting_plate_so101_v2.stl`
- `sts3215_03a_v1.stl`
- `motor_holder_so101_base_v1.stl`
- `wrist_roll_follower_so101_v1.stl`
- `moving_jaw_so101_v1.stl`
- `base_motor_holder_so101_v1.stl`
- `upper_arm_so101_v1.stl`
- `wrist_roll_pitch_so101_v2.stl`
- `under_arm_so101_v1.stl`
- `rotation_pitch_so101_v1.stl`
- `motor_holder_so101_wrist_v1.stl`
- `sts3215_03a_no_horn_v1.stl`
- `base_so101_v2.stl`
- `moving_jaw_so101_gripper_v1.stl`
- `wrist_roll_follower_so101_camera_mount.stl`

The pinned Menagerie gripper also references these mesh collision components:

- `wrist_roll_follower_so101_gripper_part0_v1.stl`
- `moving_jaw_so101_gripper_part0_v1.stl`
- `moving_jaw_so101_gripper_part1_v1.stl`

These upstream mesh names are recorded for provenance. The Phase 2A browser-validation package does not bundle those meshes.

## Model adaptation for Phase 2A

The Phase 2A MJCF retains the source six-joint articulated-chain transforms, joint axes/ranges, explicit inertials of `base`, `shoulder`, `upper_arm`, `lower_arm`, `wrist`, `gripper`, and `moving_jaw_so101_v1`, MuJoCo timestep/integrator/solver settings, position-actuator mappings/control ranges, and a subset of the source primitive collision geometry sufficient for articulated-plant validation.

The adaptation is intentionally **not** a byte-for-byte or dynamically complete copy of the Menagerie model:

- visual STL meshes are omitted;
- mesh-based gripper collision pieces are omitted;
- the source `camera_mount` child is omitted, including its camera, primitive camera collision boxes, and the `0.012 kg` mass assigned to its visual mesh in the pinned source;
- at least one broad source fixed-jaw collision primitive and other geometry not required for Phase 2A articulation validation are omitted;
- a simple `base_proxy` box is added as an estimated Phase 2A diagnostic proxy.

Because the source camera-mount mass is omitted, browser/native parity for this package proves agreement for the **RoboBuddy Phase 2A adapted plant**, not numerical equivalence to the complete Menagerie SO-101 model. Restoring or otherwise source-faithfully representing omitted payload/collision dynamics is required before manipulation or hardware-alignment claims are made.

The world frame is MuJoCo's right-handed +Z-up frame. Linear units are metres, mass is kilograms, time is seconds, and joint/control angles are radians. Every modeled revolute joint uses local axis `0 0 1`; the body transforms in the MJCF establish the corresponding physical axis in parent/world coordinates.

Joint order and ranges:

| Order | Joint | Range rad | Actuator control range rad |
| ---: | --- | --- | --- |
| 1 | `shoulder_pan` | -1.91986 .. 1.91986 | -1.91986 .. 1.91986 |
| 2 | `shoulder_lift` | -1.7453293 .. 1.7453293 | -1.74533 .. 1.74533 |
| 3 | `elbow_flex` | -1.69 .. 1.69 | -1.69 .. 1.69 |
| 4 | `wrist_flex` | -1.658063 .. 1.658063 | -1.65806 .. 1.65806 |
| 5 | `wrist_roll` | -2.7438473 .. 2.7438473 | -2.74385 .. 2.84121 |
| 6 | `gripper` | -0.174533 .. 1.7453292 | -0.17453 .. 1.74533 |

The pinned Menagerie actuator control maximum for `wrist_roll` is 2.84121 rad while the joint range maximum is 2.7438473 rad. RoboBuddy's generic command layer therefore validates both the actuator control range and the joint range; a controller target is never permission to write `qpos` directly.

## Parameter evidence classes

### SOURCE-DERIVED

- six-actuated-joint hierarchy and body transforms retained in the Phase 2A package;
- joint names, order, local axes, and joint limits;
- explicit masses, inertial-frame positions, and full inertia tensors for the retained articulated bodies listed above;
- 0.005 s fixed MuJoCo timestep;
- `implicitfast` integrator and the pinned solver settings;
- one position actuator per modeled joint and the pinned actuator control ranges;
- primitive arm/gripper collision geometry copied from the pinned Menagerie model where retained;
- gripper mechanism topology: one fixed jaw plus one actuated moving jaw;
- source camera-mount visual-mesh mass value of `0.012 kg` as an upstream fact, although that child is not represented in the Phase 2A adapted MJCF.

### ESTIMATED

The pinned Menagerie file explicitly states that its STS3215 position gains are not a one-to-one mapping of LeRobot servo gains and that they were calculated assuming a proportional gain of 16. Therefore the following source-provided simulation values are treated as estimated physical parameters rather than authoritative hardware measurements:

- `kp = 998.22`;
- `kv = 2.731`;
- `forcerange = -2.94 .. 2.94`;
- STS3215 damping `0.60`, friction loss `0.052`, and armature `0.028` for hardware-fidelity claims;
- the Phase 2A-only `base_proxy` box;
- any visual/collision completeness implied by the primitive-only diagnostic adaptation.

### CALIBRATION-REQUIRED

Before hardware-aligned fidelity can be claimed for an assembled SO-101, independently measure or verify at least:

- actual joint zero/calibration offsets and usable limits;
- installed servo model, firmware/configuration, gain and latency behavior;
- torque-speed/effort behavior and sustained/transient limits;
- backlash, friction, compliance, and mechanical play;
- gripper opening mapping, contact friction, grip force, and fingertip geometry;
- mount, tool/camera configuration, payload mass/centre of mass, and work-surface properties;
- model-to-hardware trajectory error across the intended task range.

Before Phase 2B manipulation fidelity is claimed for the simulator itself, restore or explicitly re-model and validate the omitted source payload/collision features that can affect gripper and contact dynamics, including the camera-mount contribution if that is the target hardware variant.

## Phase 2A limitations

This package is intended only to prove model-driven loading, deterministic reset, bounded actuator-target motion, multi-joint numerical stability, and browser/native MuJoCo conformance for the adapted Phase 2A plant. It does not establish hardware fidelity or complete Menagerie-model equivalence. It intentionally does not implement grasping, block transfer, bottle/glassware manipulation, task completion logic, or production Python/WebMCP routing; those remain Phase 2B work.
