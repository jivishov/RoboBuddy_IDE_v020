# OpenArm contact/workcell and WebMCP revision 3

## What changed

The rendered OpenArm now follows authoritative MuJoCo body poses, including both passive/actuated fingers individually. The same convex-component surfaces are used for rendering and collision: 36 source robot mesh components and 55 baseline physical geometries. There is no second canonical joint animation, visual-only palm offset, object weld or attachment.

The old pickup placement overlapped source-matched palms by approximately 20–25 mm. Pickups have been repositioned, the tool approach raised, and the dry transfer reference rebuilt. Source tables/supports, destination supports and the robot mount now form a coherent, physically represented workcell. Shoulder height is preserved rather than hiding a frame error by lowering the robot.

The worker refreshes derived state after every integration step, independently of sampling. Observations distinguish the requested target, moving reference, actual actuator input and measured joint state. MuJoCo contact forces and penetration are available for inspection. Grasp evaluation requires sustained bilateral contact; excessive vessel penetration or palm contact invalidates the reference task. The two vessels must actually lift, carry, reach their supports, release, settle and be cleared by the robot.

## Start and control

Select OpenArm and the bimanual workcell. Run the visible Python reference to perform the dry transfer. The 17-stage reference is 26.3 simulation seconds; slow computers may take longer in wall time. The physical Python limit for this workspace is 180 seconds. Existing project drafts may need the normal workspace reset/reload to receive the revised reference.

Turn **Agent Assist** on to register the OpenArm tools. Controls are simulation-only. Angle targets are radians; position is metres in a right-handed Z-up world. Quaternion order is WXYZ. The renderer converts to the IDE's millimetre/Y-up display once; agents should not do that conversion themselves.

`control_openarm_simulation` keeps schema `robobuddy.openarm.physical.v1` and supports `set_joint_targets`, `move_tool`, `advance` and explicit `reset`. The published JSON Schema accepts valid inputs and still rejects unknown fields. A command being accepted does not mean the target was attained.

The reference speed limits are 0.75 rad/s for arm joints and 0.60 rad/s for finger reference motion, with quintic interpolation. An explicitly requested duration that is too short is rejected before latching any target. These are controller design choices, not guarantees about actual joint velocity or calibrated hardware limits. Gripper joint1 is positive on the left and negative on the right; magnitude 0.65 rad is the reference open setting. The fingers are not independently commanded.

`move_tool` and program `tool` targets address the **pinch reference** at local EE point [-0.00143, 0, -0.153] m. Read the observed reference first. This is not an object socket, contact guarantee or automatic attachment. Position-only IK leaves orientation unconstrained. A normalized optional quaternion constrains EE orientation. IK operates on separate scratch data and never overwrites the plant state. It is not a collision-free path planner.

## Equipment construction

`inspect_openarm_workcell` returns the scene revision, body/geometry IDs, observed pinch poses, per-finger contacts, equipment affordances, passive equipment joints, transient staging and program progress. These are simulator ground-truth observations, not sensor-realistic hardware data.

`manage_openarm_workcell` uses `robobuddy.openarm.equipment.v1`:

1. `stage` supplies `expected_scene_revision` from inspection and a complete custom `equipment` array. A visible wireframe preview is shown; physics remains unchanged.
2. `apply` supplies the returned `stage_id` and `acknowledge_reset:true`. The candidate model must compile and pass initial collision checks before replacing the active session. Failed application preserves the old world. Applying is a logged setup/reset, never credited as a task outcome.
3. `discard` removes a preview. Staging/applying an empty list removes all custom equipment; baseline fixtures/vessels are retained.

Each item has an identifier, kind and `position_m` describing **bottom-centre**. `dimensions_m` contains full X width, Y width and Z height, not half-sizes. Optional yaw is in radians about world Z. Supported kinds are platform, tray, rack, vial, block, button and assembly. Assemblies accept at most 12 box/sphere/cylinder parts with local positions and bounded dimensions. Trays/racks use separated components so their cavities are not solid convex hulls. Vials are solid dry exterior models.

Limits include 12 equipment items, 128 compiled geometries, 24 kB equipment input, 0.005–2 kg dynamic item mass, and a bounded table workspace. Raw XML, arbitrary scripts, remote asset URLs and physics disabling are not accepted. The scene compiler revalidates on the worker side.

Example staging payload (replace the revision with the latest inspection value):

```json
{
  "schema_version": "robobuddy.openarm.equipment.v1",
  "command": "stage",
  "expected_scene_revision": "phase5a-openarm-v2-bimanual-stack-v3",
  "equipment": [
    {"id":"button1","kind":"button","position_m":[0.57,0.32,1.005]},
    {"id":"tray1","kind":"tray","position_m":[0.35,-0.38,1.005]},
    {"id":"vial1","kind":"vial","position_m":[0.35,-0.38,1.08]}
  ]
}
```

The vial deliberately begins above the tray and settles through gravity/contact after simulated time is advanced. The button is a passive spring-cap mechanism, not a real electrical/thermal instrument.

## Bounded programs and outcome checks

`run_openarm_program` uses `robobuddy.openarm.program.v1`. Provide the current `expected_scene_revision` and 1–32 `segments`. It executes the current scene without an implicit reset. Worst-case simulated duration, including waits, is limited to 60 seconds; wall time to 180 seconds. Each segment has `duration_seconds` and optionally `targets_rad` OR `tool`. A segment without either is a bounded dwell.

Optional `wait_for` predicates are `bilateral_grasp`, `released`, `supported`, `in_region`, `tool_reached`, and `equipment_joint`. They use actual named observations and sustained dwell (default 0.06 s), with a finite additional timeout (default 3 s; at most 5 s). A timeout stops the sequence and returns the completed/failed stage instead of reporting success. A program with no outcome predicates reports execution completion only. Reference flask/beaker task results are separate and do not certify arbitrary custom equipment tasks.

For the example button, use the following safe approach from the initial scene. Every tool target is left-sided with orientation `[0.7071067811865476,0,-0.7071067811865476,0]`:

| Stage | Target / operation | Duration |
|---|---|---:|
| Lift clear of the flask before moving laterally | Tool [0.554,0.1535,1.25] | 4 s |
| Move above the created button | Tool [0.57,0.32,1.23] | 4 s |
| Close the empty gripper | `openarm_left_finger_joint1:0.02` | 3 s |
| Approach | Tool [0.57,0.32,1.060] | 4 s |
| Press | Tool [0.57,0.32,1.050] | 2 s |
| Withdraw | Tool [0.57,0.32,1.150] | 2 s |

On the press stage, wait for `equipment_joint`, `joint_id:"lab_button1_press"`, `minimum:0.004`, `maximum:0.006`. On withdrawal, require 0–0.001 m travel. The state changes must come from finger contact and passive spring return. `supported` for object `lab_vial1` and support geometry `lab_tray1_base` verifies tray support. This exact approach is included in the worker/browser regression tests; it should not be generalized to changed equipment positions without checking reach and clearance.

Human Stop, workspace switching, cancellation or revoking Agent Assist prevents subsequent program steps from moving a replacement session. Direct controls and programs acquire the same UI execution lease used to prevent simultaneous Python ownership. Pausing a program freezes its simulation scheduler, not its physical velocities.

## Verification and limits

Focused commands:

```sh
node tests/physics/openarm-phase5a-core.mjs
node tests/physics/openarm-worker.mjs
python tests/validate_openarm_schemas.py
python native/openarm_v2_reference.py --trial nominal
python native/openarm_v2_reference.py --trial nominal --timestep 0.0005
npx playwright test tests/openarm-phase5a-browser.spec.mjs tests/openarm-workcell-browser.spec.mjs
```

Negative native profiles: `miss-left`, `weak-grip`, `blocked-left`, `outside-left`, `insufficient-budget`. They are explicit test-only interventions, never hidden controller assists.

The worker test uses the real bundled MuJoCo 3.11.0 WebAssembly runtime, not mocked dynamics. It verifies command atomicity, observation purity, sampling independence, IK without plant writes, nominal transfer, initial overlap rejection, robot-driven passive button motion, spring return and vial/tray contact. Native tests are a separate implementation comparison, not independent proof of hardware truth. Browser tests exercise both the full IDE/Python path and tool registration through a test adapter; they do not claim that CI Chromium implements native experimental WebMCP.

An observed successful reference run does not establish calibrated hardware fidelity or universal grasp reliability. Contact meshes remain per-component convex approximations. Friction, vessel mass/inertia, grip operating limits, servo bias assistance and equipment parameters are explicitly simulator estimates/design choices. No fluids, heat, chemistry, tactile-sensor realism, glass compliance, instrument measurement physics, hardware transport or guaranteed collision-free trajectories are included. Generated assets and source lineage are detailed in `models/openarm_v2/PROVENANCE.md`.
