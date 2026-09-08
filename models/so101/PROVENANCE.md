# SO-101 Phase 2A model provenance

This package is a controlled articulated-plant validation model for RoboBuddy IDE Physics Preview Phase 2A. It is not a hardware-calibrated digital twin, it is not dynamically identical to the complete pinned MuJoCo Menagerie package, and it is not yet the production SO-101 workspace.

## Authoritative sources and adaptation basis

The physically executed Phase 2A model is based on a pinned Google DeepMind MuJoCo Menagerie SO-101 simulation package. Menagerie in turn records its derivation from The Robot Studio's public SO-101 simulation model. Both layers are pinned because Menagerie intentionally changes simulation details relative to the original project model.

- Primary executed-model source: Google DeepMind MuJoCo Menagerie, `robotstudio_so101`.
- Pinned Menagerie revision: `8161bba264d7fa7c99ca301e91e7fb44737676ad`.
- Menagerie source MJCF: `robotstudio_so101/so101.xml`.
- Original project source: The Robot Studio `SO-ARM100`, `Simulation/SO101/so101_new_calib.xml`.
- Original project revision recorded by Menagerie: `aec17bbc256d1a7342d53aaa4950595d4c30b40d`.
- Source variant: SO-101 follower arm using STS3215-class servos and a single moving-jaw gripper.
- RoboBuddy Phase 2A adaptation: the six-actuated-joint chain from the pinned Menagerie model, with a deliberately reduced self-contained geometry set and without the Menagerie `camera_mount` child.
- License: Apache License 2.0, as declared by the pinned Menagerie SO-101 package.

Source URLs:

- https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101
- https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/so101.xml
- https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/LICENSE
- https://github.com/TheRobotStudio/SO-ARM100/blob/aec17bbc256d1a7342d53aaa4950595d4c30b40d/Simulation/SO101/so101_new_calib.xml

Menagerie's pinned README says its derivation steps include copying the public SO-101 MJCF, switching to `implicitfast`, changing the actuator force-range handling, adding primitive arm/gripper collision geometry and manipulation-oriented gripper solver parameters, and adding a camera mount. Therefore `SOURCE-DERIVED` below means the parameter is traceable to the specifically named source layer; it does **not** mean every Menagerie value is an original manufacturer/hardware limit.

### Wrist-roll limit discrepancy

A direct source comparison found a meaningful limit difference that must not be hidden:

- The Robot Studio source revision recorded by Menagerie gives `wrist_roll` a range of approximately `-2.7438473 .. 2.8412063 rad`.
- The pinned Menagerie model gives `wrist_roll` a symmetric joint range of `-2.7438473 .. 2.7438473 rad` while keeping the actuator control range approximately `-2.74385 .. 2.84121 rad`.
- The Menagerie README does not list this joint-range narrowing among its high-level derivation steps.

RoboBuddy Phase 2A preserves the **pinned Menagerie joint constraint** because that is the executed simulation source used for the browser/native conformance plant. It is therefore a Menagerie-derived simulation constraint, not a claimed measured hardware limit. Before hardware alignment or task-envelope certification, the usable wrist range must be established from the actual assembled robot/calibration and reconciled with the project-source and Menagerie values.

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

The Phase 2A MJCF retains the pinned Menagerie six-joint articulated-chain transforms, joint axes/ranges, explicit inertials of `base`, `shoulder`, `upper_arm`, `lower_arm`, `wrist`, `gripper`, and `moving_jaw_so101_v1`, MuJoCo timestep/integrator/solver settings, position-actuator mappings/control ranges, and a subset of the source primitive collision geometry sufficient for articulated-plant validation.

The adaptation is intentionally **not** a byte-for-byte or dynamically complete copy of the Menagerie model:

- visual STL meshes are omitted;
- mesh-based gripper collision pieces are omitted;
- the source `camera_mount` child is omitted, including its camera, primitive camera collision boxes, and the `0.012 kg` mass assigned to its visual mesh in the pinned source;
- at least one broad source fixed-jaw collision primitive and other geometry not required for Phase 2A articulation validation are omitted;
- a simple `base_proxy` box is added only as an estimated diagnostic visual proxy; it is explicitly non-colliding (`contype=0`, `conaffinity=0`) and therefore cannot create a source-unsupported contact force.

Because the source camera-mount mass is omitted, browser/native parity for this package proves agreement for the **RoboBuddy Phase 2A adapted plant**, not numerical equivalence to the complete Menagerie SO-101 model. Restoring or otherwise source-faithfully representing omitted payload/collision dynamics is required before manipulation or hardware-alignment claims are made.

The world frame is MuJoCo's right-handed +Z-up frame. Linear units are metres, mass is kilograms, time is seconds, and joint/control angles are radians. Every modeled revolute joint uses local axis `0 0 1`; the body transforms in the MJCF establish the corresponding physical axis in parent/world coordinates.

Joint order and ranges used by the executed Phase 2A Menagerie-derived plant:

| Order | Joint | Joint range rad | Actuator control range rad |
| ---: | --- | --- | --- |
| 1 | `shoulder_pan` | -1.91986 .. 1.91986 | -1.91986 .. 1.91986 |
| 2 | `shoulder_lift` | -1.7453293 .. 1.7453293 | -1.74533 .. 1.74533 |
| 3 | `elbow_flex` | -1.69 .. 1.69 | -1.69 .. 1.69 |
| 4 | `wrist_flex` | -1.658063 .. 1.658063 | -1.65806 .. 1.65806 |
| 5 | `wrist_roll` | -2.7438473 .. 2.7438473 | -2.74385 .. 2.84121 |
| 6 | `gripper` | -0.174533 .. 1.7453292 | -0.17453 .. 1.74533 |

The command layer validates both the actuator control range and the joint range; a controller target is never permission to write `qpos` directly. Thus, a `wrist_roll` target between the Menagerie joint maximum and actuator control maximum is rejected by the Phase 2A contract even though it falls within the actuator's declared control range.

## Parameter evidence classes

### SOURCE-DERIVED

- six-actuated-joint hierarchy and body transforms retained from the pinned Menagerie model;
- joint names, order, local axes, and the **Menagerie simulation joint limits** used by this Phase 2A plant;
- explicit masses, inertial-frame positions, and full inertia tensors for the retained articulated bodies listed above;
- 0.005 s fixed MuJoCo timestep;
- `implicitfast` integrator and the pinned solver settings;
- one position actuator per modeled joint and the pinned actuator control ranges;
- primitive arm/gripper collision geometry copied from the pinned Menagerie model where retained;
- gripper mechanism topology: one fixed jaw plus one actuated moving jaw;
- source camera-mount visual-mesh mass value of `0.012 kg` as a Menagerie fact, although that child is not represented in the Phase 2A adapted MJCF.

### ESTIMATED

The pinned Menagerie file explicitly states that its STS3215 position gains are not a one-to-one mapping of LeRobot servo gains and that they were calculated assuming a proportional gain of 16. Therefore the following source-provided simulation values are treated as estimated physical parameters rather than authoritative hardware measurements:

- `kp = 998.22`;
- `kv = 2.731`;
- `forcerange = -2.94 .. 2.94`;
- STS3215 damping `0.60`, friction loss `0.052`, and armature `0.028` for hardware-fidelity claims;
- the visual-only Phase 2A `base_proxy` shape;
- any visual/collision completeness implied by the primitive-only diagnostic adaptation.

### CALIBRATION-REQUIRED

Before hardware-aligned fidelity can be claimed for an assembled SO-101, independently measure or verify at least:

- actual joint zero/calibration offsets and usable limits, explicitly including reconciliation of the `wrist_roll` Menagerie/project-source discrepancy;
- installed servo model, firmware/configuration, gain and latency behavior;
- torque-speed/effort behavior and sustained/transient limits;
- backlash, friction, compliance, and mechanical play;
- gripper opening mapping, contact friction, grip force, and fingertip geometry;
- mount, tool/camera configuration, payload mass/centre of mass, and work-surface properties;
- model-to-hardware trajectory error across the intended task range.

Before Phase 2B manipulation fidelity is claimed for the simulator itself, restore or explicitly re-model and validate the omitted source payload/collision features that can affect gripper and contact dynamics, including the camera-mount contribution if that is the target hardware variant.

## Phase 2A limitations

This package is intended only to prove model-driven loading, deterministic reset, bounded actuator-target motion, multi-joint numerical stability, and browser/native MuJoCo conformance for the adapted Phase 2A plant. It does not establish hardware fidelity, manufacturer-limit fidelity, or complete Menagerie-model equivalence. It intentionally does not implement grasping, block transfer, bottle/glassware manipulation, task completion logic, or production Python/WebMCP routing; those remain Phase 2B work.
