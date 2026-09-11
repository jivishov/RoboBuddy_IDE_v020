# Asimov 1 physical integration

Source: `menloresearch/asimov-1@732cc60dcb8f2b4fd26c3d7346b35f9b89c3cd47`.
Entry point: choose **Asimov 1 — Physical** in the robot selector. Default task is
**Mounted Joint Lab**; **Free-base Dynamics** and **Passive Gravity Drop** are separate tasks.
The original RoboBuddy_IDE and RoboBuddy_AI repositories are not modified.

## What is represented

The official MJCF defines 23 hinge joints, a floating pelvis, 26 robot bodies,
25 unique STL files, fixed neck yaw/pitch bodies, and 32.22491296549718 kg of
model mass. The hardware's advertised 25 joints and approximate 35 kg are not
substituted for those source values. Both elbows retain their source reference
offsets. Joint control uses the exact source names, axes and radians.

`generate_asimov_models.py` pins the input MJCF and URDF by Git blob hash. It
retains all source inertials, joint kinematics, armature/damping, contact
primitives, exclusions, sensors, floor friction and solver settings. The timestep
is 0.005 s, implicitfast/Newton, 10 iterations, 20 line-search iterations, with
source noslip/impratio/cone parameters unchanged. The eight original foot spheres
remain. Zero-density/non-colliding visual geoms are removed from the physics XML;
all 25 full-resolution STL files are losslessly gzip-compressed for presentation.
Compressed and decompressed hashes are checked before rendering. No mesh is simplified.

The source defines no actuators. This integration adds one **ideal joint torque
motor** per hinge, bounded in command and effort by the URDF limits. PD gains are
**repository estimates**, not Menlo's training configuration and not measured
motor parameters. Ankle pitch/roll remain the source's ideal coordinates, not the
hardware parallel ankle transmission. The wrist-end meshes have no finger actuators;
no grasping capability is claimed. The two neck bodies stay fixed.

## Authority and controls

One PhysicsSession and the fleet's BrowserMuJoCoBackend own each active workspace.
The Asimov worker uses the existing pinned MuJoCo 3.11.0 WASM. Ordinary control
writes only `data.ctrl`; it never writes root/joint position or velocity. Explicit
reset calls the model's named initial keyframe. An explicit, logged torque-off
setup is available internally/Python but not to the agent control tool.

The renderer consumes all observed body poses in metres/Z-up, with a single
presentation-only conversion to Three.js Y-up. It does not integrate, ground-snap,
set joint state, advance simulation, or claim a requested target was achieved.
Python uses `robobuddy.sim`, `set_joint_targets`, `wait_sim` and `get_state`.
WebMCP `control_asimov_physical_simulation` remains opt-in with Assist access,
workspace/epoch ownership, cancellation and bounded steps/time. It exposes inspect,
state, joint targets, advance, stop and reset. It exposes no stand, walk, grasp,
root-pose/velocity write, upright correction, force or object-placement operation.

## Declared scenes and limitations

- **Mounted:** pelvis fixed by the model at 1.05 m; a visible, non-contact mount
  documents the fixture. The source-adjacent pelvis pairs are excluded to preserve
  the same adjacent-body collision filtering when MuJoCo welds the root to world.
  This is a joint dynamics laboratory, not evidence of standing.
- **Free-base:** source pelvis height 0.630 m and source reference joint positions;
  estimated bounded PD attempts to hold these positions. It can genuinely fall.
- **Drop:** initial pelvis height 0.95 m with zero commanded torque. The fall and
  subsequent contact are physical; reset is not a recovery action.

No trained walking policy, verified standing/balance recovery, hardware connection,
real actuator calibration, backlash, compliance, motor current/thermal dynamics,
or physical robot comparison is supplied. Evaluation is observation-only; the
application does not manufacture success events.

## Validation

Run from the repository root:

```sh
ASIMOV_WASM_REPORT=/tmp/asimov-wasm.json node tests/physics/asimov-worker.mjs
python tests/validate_asimov_native.py --wasm-report /tmp/asimov-wasm.json
npx playwright test tests/asimov-physical-browser.spec.mjs --workers=1
```

The WASM gate covers all three variants, joint tracking, torque caps, actual contact
forces, passive fall, exact reset, pause and invalid requests (including numeric
strings, unknown joints, NaN/Infinity, unknown command fields and unsupported
standing). Malformed commands must preserve the previous command. Corrupted XML
must fail loading and leave no live plant.

Native conformance uses MuJoCo 3.11.0, the same 4-step observation cadence, and
compares every joint, observed body and contact normal force. With `--upstream`,
the full original STL-bearing source is compiled and compared against the stripped
physics model for all body inertias/transforms, joints, armature and damping.
The browser suite additionally covers session/Python bridge ownership, rendered
source mesh/body alignment, no frame-driven stepping, IDE selectors/live Python,
opt-in agent controls and switching back to the existing G1 workspace.

Local native/shipped-WASM validation on 2026-09-11 passed 208 sample comparisons.
Worst joint difference was 5.14e-16 rad, body difference 2.23e-16 m, contact force
difference 4.46e-12 N. Source compiled physical arrays matched within 1e-12.
These are **numerical checks**, not hardware validation. Browser results are
reported by the separate CI job; local browser networking was unavailable.

## Source licensing

Preserved verbatim under `models/asimov/source/`:
`HARDWARE-LICENSE.txt` (CERN-OHL-S-2.0), `SOFTWARE-LICENSE.txt` (GPL-2.0), original
MJCF, original URDF and generated source audit. Model/mesh derivations are not
relicensed as MIT. Original RoboBuddy glue/fixtures remain separately scoped by
the repository license. Upstream source and corresponding changes are publicly
available in this repository and in the pinned upstream repository.
