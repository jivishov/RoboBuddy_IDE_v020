# Programming Asimov through WebMCP

Simulation only. This changes only `jivishov/RoboBuddy_IDE_v020`. No physical robot, remote hardware connection or paid simulation service is used.

## Workspaces

Select **Asimov 1 — Physical**. The original six workspaces remain. Three additive workspaces use the unchanged free-base MuJoCo model:

| Workspace ID | Balance observations | Additional assumption |
|---|---|---|
| `asimov-sensor-standing` | Synthetic body-frame gyro and normalized projected gravity; 10 ms feedback delay | Independent source-equivalent ankle axes |
| `asimov-sensor-standing-delay` | Same sensors, 25 ms feedback delay | Not a command-transport-delay test |
| `asimov-sensor-standing-ankle-stress` | Same 10 ms sensors | Hypothetical ankle losses, 70% source joint torque caps and motoring taper |

Every new workspace has an executable async Python starter requesting 12 simulation seconds. The balance regulator runs at 200 Hz; physics and standing assessment at 400 Hz. A 30 ms exponential smoothing time constant is explicit. The evaluator retains the original 1 s settling plus 10 s continuous standing criteria. Later falls invalidate earlier passes. Reset is explicit, not a recovery.

The body sensor profile is `asimov-body-sensors-v2`; the regulator is `asimov-sensor-stance-v2`. Gyro and projected gravity share the `pelvis_link` frame. Sensor histories advance only on simulation-time sample ticks. Cold-start history is unavailable; stale/invalid observations after a bounded warmup latch a sensor fault. There is no ground-truth attitude fallback in this regulator. The inner joint PD loop still uses ideal local encoder feedback; this is not a fully calibrated hardware control stack. Projected gravity is synthetic attitude-derived data, not a raw accelerometer or a full IMU estimator.

## Enable agent control

Choose **Agent Assist** in the existing opt-in controls. The registered tool remains `control_asimov_physical_simulation`, with `schema_version: "robobuddy.asimov.physical.v1"`. The additive programming contract is `robobuddy.asimov.program.v1`.

Start with `inspect_capability`. It reports all 23 named joint ranges, the selected timestep/controller, unsupported operations, mass discrepancy, limits and evidence. Source effort limits in the joint list are not the instantaneous actuator caps; inspect `actuator_model` for the actual selected experiment.

| Command | Effect |
|---|---|
| `inspect_capability` | Read model/joint/programming metadata; starts nothing |
| `read_state` | Ground-truth simulated state and independent standing assessment |
| `read_sensors` | Synthetic measurements only; unsupported in original reference scenes |
| `set_joint_targets` | Bounded targets in radians; releases a standing controller |
| `engage_stand` | Engage the controller declared by the selected standing workspace; no time advance |
| `set_standing_targets` | Bounded waist/arm targets while keeping an already active nonfailed standing controller |
| `advance` | At most 2 simulation seconds per call |
| `run_sequence` | 1–32 prevalidated segments, at most 2 s each and 12 s total |
| `wait_for_joint` | Observe existing control until a measured position remains within tolerance, or timeout |
| `stop` | Bounded measured-position hold; not a velocity reset |
| `reset` | New initial condition and trial; not task success |

Waist/arm targets do not provide whole-body motion planning and can cause a fall. The 12 leg joints (including the four ankle axes) cannot be overridden through `set_standing_targets`. Full joint control remains available through ordinary targets, with balance explicitly released.

## Example: stand, then move the elbows without releasing balance

Select `asimov-sensor-standing`, reset explicitly, then send:

```json
{"schema_version":"robobuddy.asimov.physical.v1","command":"engage_stand"}
```

```json
{
  "schema_version": "robobuddy.asimov.physical.v1",
  "command": "run_sequence",
  "segments": [
    {"duration_seconds": 1.0},
    {"targets_rad": {"left_elbow_joint": 1.0, "right_elbow_joint": -1.0}, "preserve_standing": true, "duration_seconds": 0.5},
    {"targets_rad": {"left_elbow_joint": 0.9, "right_elbow_joint": -0.9}, "preserve_standing": true, "duration_seconds": 0.5}
  ]
}
```

A completed sequence means the commands/time segments executed—not that standing passed, a pose was achieved, or a physical robot could do the same. This two-second example is shorter than the 11 seconds needed for the standing gate. Continue with bounded advances and inspect the assessment. Use duration-only segments to leave all targets/controllers unchanged.

## Example: measured-position wait in the mounted lab

```json
{"schema_version":"robobuddy.asimov.physical.v1","command":"set_joint_targets","targets_rad":{"left_elbow_joint":1.0},"max_steps":4000}
```

```json
{
  "schema_version": "robobuddy.asimov.physical.v1",
  "command": "wait_for_joint",
  "joint_id": "left_elbow_joint",
  "target_rad": 1.0,
  "tolerance_rad": 0.03,
  "dwell_seconds": 0.1,
  "timeout_seconds": 2.0
}
```

The wait does not send a target. It uses ground-truth joint position, checked every physics step, and returns `achieved` or `timeout`. A successful API response can therefore contain `achieved: false`. The target must be feasible under the actual plant. Closed-loop agents may also read synthetic sensors between bounded operations and choose their next commands. Existing editor/program-run WebMCP functions remain available for async Python programs; no separate unbounded code interpreter was added to this tool.

## Bounds and cancellation

Inputs and the complete sequence are validated before any mutation, including joint ranges, controller transitions and timestep alignment. Durations must be multiples of 5 ms in original reference scenes or 2.5 ms in actuator/sensor scenes. A single call has a fixed 120 s wall deadline, never extended by progress. Physics advancement checks cancellation, access, workspace and session ownership between at-most-50-ms simulation batches. An already submitted batch cannot be undone. The normal UI Stop remains available; a second concurrent robot-control call is rejected.

No command sets root pose/velocity, applies external force, changes model mass/contact properties, invents a walking gait, silently resets after failure, or connects to hardware. Model sensitivity variations are explicit test-runner cases or separately selected immutable workspaces, not hidden changes from an agent.

## Verification and evidence

Run the original suites plus:

```sh
node tests/physics/asimov-sensor-core.mjs
node tests/physics/asimov-program-core.mjs
ASIMOV_SENSOR_REPORT=/tmp/asimov-sensor-wasm.json node tests/physics/asimov-sensor-worker.mjs
python tests/validate_asimov_sensors.py --wasm-report /tmp/asimov-sensor-wasm.json --report /tmp/asimov-sensor-validation.json
npx playwright test tests/asimov-physical-browser.spec.mjs --workers=1
```

The native test independently reimplements the sensor/controller and compares with actual WASM traces. Additional cases vary seed, feedback delay, ankle losses, mass, COM, floor/foot friction and contact softness. Uniform mass/inertia scaling to 35 kg is explicitly a hypothetical test, never a correction to the production model. Combined cases and negatives are included. Reports preserve failures rather than tuning every case to pass. CI artifacts are retained 90 days; a compact report should accompany each accepted release.

## Evidence-dependent work deliberately not invented

Actual paired ankle ratios, motor-space limits and consistent reflected inertia still require compatible numeric evidence. So do fitted torque–speed/braking curves, breakaway friction, electrical/thermal peak limits, true encoder/CAN timing and hardware validation. The independent-axis ankle stress profile tests dependence on assumptions but does not solve paired-motor feasibility. No walking, grasping or general recovery capability is claimed.
