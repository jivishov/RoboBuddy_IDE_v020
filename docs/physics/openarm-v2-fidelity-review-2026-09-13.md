# OpenArm V2 fidelity review — 2026-09-13

## Scope

This review covers the browser MuJoCo OpenArm V2 bimanual workspace in RoboBuddy IDE v0.2.0. It distinguishes source-derived robot properties, RoboBuddy benchmark assumptions, presentation-only geometry, and hardware validation.

## Findings and changes in this revision

### 1. Grasp contact geometry

The pinned upstream `enactic/openarm_mujoco` V2 model uses four collision mesh parts per finger (`finger_inner_part_00..03.stl` and `finger_outer_part_00..03.stl`). RoboBuddy currently uses repository-local 8 mm capsule surrogates instead of those meshes, so exact finger/palm clearance is not yet represented.

An experimental revision attempted to shift the capsule centerline using the finger body's inertial center-of-mass offset. Native MuJoCo regression testing showed that this destabilized the nominal flask manipulation. That experiment was reverted: an inertial COM is not a justified substitute for collision-mesh geometry. The physical model therefore remains on the previously validated primitive-proxy baseline until the pinned upstream collision meshes can be bundled and audited properly.

This revision improves fidelity without inventing a new geometry calibration by adding two independent task-level physical gates:

- signed fingertip/vessel contact distance is tracked and task success rejects a grasp whose maximum penetration exceeds 4 mm;
- any vessel contact with the corresponding `*_ee_proxy` palm/end-effector collider invalidates grasp clearance and task success.

These are physical-state checks from MuJoCo contact pairs. They do not alter object transforms, attach objects to the gripper, or manufacture successful grasps. They make the current approximation fail visibly when it produces an obviously embedded/invalid contact instead of counting that contact as a valid manipulation.

### 2. Scene layering

The old Three.js world-floor grid was rendered at presentation `y=0` while the physical tabletop is approximately `z=1.005 m` in MuJoCo. The result looked like a second detached floor one metre below the robot workcell.

The grid is presentation-only and now sits 1 mm above the actual tabletop (`1006 mm` in the Three.js presentation frame) to avoid z-fighting. The physical table height, robot mount, gravity, and collision world are unchanged. The default camera was also moved closer to the workcell.

### 3. Physical operation review

#### Stronger / source-derived parts

- V2 bimanual kinematic tree, mirrored joint axes and ranges.
- Source-transcribed link inertials.
- Source-simulation position-controller gains/limits and legitimate mechanical finger couplings.
- One MuJoCo `PhysicsSession` is authoritative for both arms and all free rigid bodies.
- Flask and beaker are free bodies. Task execution does not write their qpos/qvel, parent them to fingers, weld them, snap them to targets, or advance them in presentation code.
- Task evaluation uses observed bilateral contact, lift, carried motion, support-while-held, release, settling, retreat, contact penetration, and palm-clearance evidence.

#### Still estimated / not hardware-calibrated

- Link and gripper collision geometry remains primitive rather than the upstream mesh collision set. Same-arm self-collision and finger/palm clearance therefore remain approximate rather than being advertised as exact.
- Flask, beaker, hotplate, ring stand, table, material friction, dry mass/inertia and contact parameters are benchmark values, not measurements from an installed workcell.
- Position actuators model the published simulator interface; they do not reproduce motor firmware, backlash, structural compliance, fingertip material compliance, electrical limits, thermal limits, encoder noise, communication delay, or a calibrated force-control loop.
- No tactile sensor or measured grasp force exists. A bilateral contact is a simulator contact condition, not proof of a safe real-world grasp.
- No liquid, filtration, heating, glass deformation/breakage, or thermochemical process is simulated. This remains a rigid-body dry manipulation task.
- Hardware alignment remains `calibration-required` until real V2 joint-zero, tool-frame, timing, payload and grasp/trajectory measurements are compared with the model.

### 4. WebMCP laboratory construction and programming

Two bounded OpenArm-specific WebMCP capabilities are added:

1. `configure_openarm_lab_equipment`
   - accepts only validated box/cylinder rigid-body primitives;
   - each item is either a fixed fixture or a free rigid body;
   - maximum 12 items / 6 free bodies;
   - positions, full extents, dimensions and masses are bounded to the OpenArm tabletop/workcell;
   - configuration recompiles a new temporary MuJoCo scene and resets the physical session;
   - no arbitrary XML, mesh URL, script, plugin, weld, source write, save/publish path, or hardware transport is exposed.

2. `program_openarm_simulation`
   - runs up to 24 bounded joint-target segments;
   - maximum 2 simulation seconds per segment / 12 seconds total;
   - target ranges come from the registered OpenArm model package;
   - durations must align to the fixed 1 ms MuJoCo timestep;
   - every segment executes against the same authoritative physical session and cannot directly move task objects.

This provides a practical WebMCP path for an agent to construct simple lab fixtures/rigid objects and then program the OpenArm around them without turning WebMCP into an arbitrary scene-code or physics transport surface.

## Highest-value next fidelity steps

1. Bundle the pinned upstream OpenArm V2 collision meshes and replace the finger/link primitive proxies after browser performance and asset-license review. The browser MuJoCo backend already has a precedent for loading repository assets through a virtual filesystem in other robot integrations; applying that approach to the pinned OpenArm collision parts is the clearest next improvement for finger/palm clearance and self-collision fidelity.
2. Add a measured OpenArm V2 hardware calibration profile: joint-zero/tool-frame checks, slow reference trajectories, grasp opening/closing response, payload and end-effector pose error.
3. Replace the current timed reference manipulation sequence with a contact-aware grasp routine that approaches, closes in small bounded increments, stops tightening after stable bilateral contact, then verifies object motion before lift.
4. Add collision-aware Cartesian planning/IK as a controller layer. Planning must generate actuator targets; it must not become a second state authority.
5. Expand the WebMCP equipment catalog only with validated geometry semantics. Hollow vessels/rings should use appropriate compound/mesh collision geometry rather than a visually hollow object with a physically solid convex collider.

## Claim boundary

The revised OpenArm scene is a more defensible browser rigid-body simulation and agent-programmable workcell. It is not a calibrated digital twin and it does not establish safe or transferable control of assembled OpenArm hardware.