# SO-101 Phase 2A model provenance

This package is a controlled articulated-plant validation model for RoboBuddy IDE Physics Preview Phase 2A. It is not a hardware-calibrated digital twin and is not yet the production SO-101 workspace.

## Authoritative source

- Upstream package: Google DeepMind MuJoCo Menagerie, `robotstudio_so101`.
- Pinned Menagerie revision: `8161bba264d7fa7c99ca301e91e7fb44737676ad`.
- Source MJCF: `robotstudio_so101/so101.xml`.
- Menagerie provenance: derived from The Robot Studio SO-101 `so101_new_calib.xml`; the Menagerie README records copied source revision `aec17bbc256d1a7342d53aaa4950595d4c30b40d`.
- Variant: The Robot Studio SO-101 follower arm using STS3215-class servos and the single moving-jaw gripper described by the pinned Menagerie MJCF.
- License: Apache License 2.0, as declared by the pinned Menagerie SO-101 package.

Source URLs:

- https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101
- https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/so101.xml
- https://github.com/google-deepmind/mujoco_menagerie/blob/8161bba264d7fa7c99ca301e91e7fb44737676ad/robotstudio_so101/LICENSE

## Model adaptation for Phase 2A

The Phase 2A MJCF keeps the source kinematic tree, link transforms, joint axes/ranges, link masses/full inertias, MuJoCo timestep/integrator/solver settings, position-actuator mappings, and source primitive collision proxies needed for articulated validation. It intentionally omits upstream visual STL meshes and mesh-based gripper collision pieces so the browser worker can compile one self-contained MJCF without a general arbitrary-asset loader. A simple `base_proxy` box is an estimated diagnostic visual/collision proxy. No object-grasp task relies on it in Phase 2A.

The world frame is MuJoCo's right-handed +Z-up frame. Linear units are metres, mass is kilograms, time is seconds, and joint/control angles are radians. Every modeled revolute joint uses local axis `0 0 1`; the body transforms in the MJCF establish the corresponding physical axis in parent/world coordinates.

Joint order and ranges:

| Order | Joint | Range rad | Actuator |
| ---: | --- | --- | --- |
| 1 | `shoulder_pan` | -1.91986 .. 1.91986 | `shoulder_pan` |
| 2 | `shoulder_lift` | -1.7453293 .. 1.7453293 | `shoulder_lift` |
| 3 | `elbow_flex` | -1.69 .. 1.69 | `elbow_flex` |
| 4 | `wrist_flex` | -1.658063 .. 1.658063 | `wrist_flex` |
| 5 | `wrist_roll` | -2.7438473 .. 2.7438473 | `wrist_roll` |
| 6 | `gripper` | -0.174533 .. 1.7453292 | `gripper` |

The pinned Menagerie actuator control maximum for `wrist_roll` is 2.84121 rad while the joint range maximum is 2.7438473 rad. RoboBuddy's generic command layer therefore validates both the actuator control range and the joint range; a controller target is never permission to write `qpos` directly.

## Parameter evidence classes

### SOURCE-DERIVED

- robot/link hierarchy and body transforms;
- joint names, order, local axes, and joint limits;
- link masses, inertial-frame positions, and full inertia tensors;
- 0.005 s fixed MuJoCo timestep;
- `implicitfast` integrator and the pinned solver settings;
- one position actuator per modeled joint and the pinned actuator control ranges;
- primitive arm/gripper collision geometry copied from the pinned Menagerie model where retained;
- gripper mechanism topology: one fixed jaw plus one actuated moving jaw.

### ESTIMATED

The pinned Menagerie file explicitly states that its STS3215 position gains are not a one-to-one mapping of LeRobot servo gains and that they were calculated assuming a proportional gain of 16. Therefore the following source-provided simulation values are treated as estimated physical parameters rather than authoritative hardware measurements:

- `kp = 998.22`;
- `kv = 2.731`;
- `forcerange = -2.94 .. 2.94`;
- STS3215 damping `0.60`, friction loss `0.052`, and armature `0.028` for hardware-fidelity claims;
- the Phase 2A-only `base_proxy` box;
- any visual appearance implied by the primitive-only diagnostic model.

### CALIBRATION-REQUIRED

Before hardware-aligned fidelity can be claimed for an assembled SO-101, independently measure or verify at least:

- actual joint zero/calibration offsets and usable limits;
- installed servo model, firmware/configuration, gain and latency behavior;
- torque-speed/effort behavior and sustained/transient limits;
- backlash, friction, compliance, and mechanical play;
- gripper opening mapping, contact friction, grip force, and fingertip geometry;
- mount, tool/camera configuration, payload mass/centre of mass, and work-surface properties;
- model-to-hardware trajectory error across the intended task range.

## Phase 2A limitations

This package is intended only to prove model-driven loading, deterministic reset, bounded actuator-target motion, multi-joint numerical stability, and browser/native MuJoCo conformance. It does not establish hardware fidelity. It intentionally does not implement grasping, block transfer, bottle/glassware manipulation, task completion logic, or production Python/WebMCP routing; those remain Phase 2B work.
