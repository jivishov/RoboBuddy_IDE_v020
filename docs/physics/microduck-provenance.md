# MicroDuck Phase 5C physical provenance

This records the physical MicroDuck workspace. It is a **separate** record from
`docs/physics/microduck-simulator.md`, which documents the reference-aligned policy
demonstrator and remains accurate for that workspace. Neither supersedes the other: the two
workspaces coexist, carry different backends, and carry different evidence labels.

## 1. Pinned sources

Phase 5C is a matched-environment integration, so it pins two sources. Neither is authority
for everything.

| Role | Source | Revision |
|---|---|---|
| Deployed runtime and controller | `pollen-robotics/microduck` | `590b986bd8c0d50ae02cb3ea2f59c463b6828168` |
| Physical / RL environment | `pollen-robotics/microduck_rl` | `519142b1f5bf59fdfd44d06c205119e7fff8e3cb` (2026-07-27, "kick") |
| Policy byte origin (recorded, not inspected) | `apirrone/microduck_runtime` | `5f3b314` (`roulade.onnx` at `7e4ab6d`) |

The deployed source supplies the policy bytes, the 61-value observation contract, the joint
order, the home pose, the command encoding, the skill priority chain, the action scaling, the
target filters and the 50 Hz cadence. The RL source supplies the collision set, the identified
servo model, the physics timestep, the control decimation, the reset pose and the ball prop.
None of those five exist in the deployed source at all.

### Why this RL revision

`microduck_rl` develop head does **not** match the deployed robot. The deployed
`kinematics/assets/alpha/robot_walk.xml` is the RL model over a bounded window:

* the window opens at `659429207558d69a6270560e33725b84f26a954e` (2026-07-01, "update model
  (head_pitch in right direction)");
* `1ee2e3c05b83348e929d617fcf8d5c9f73503fe7` (2026-07-28, "update robot model with more
  accurate weights") re-weights the model and is outside it;
* the selected pin `519142b` is the newest revision on that line that still carries the
  deployed model **and** already carries the source ball prop, so the walking model, the
  ground-contact collision model, the velstand configuration and the kick prop all come from
  one revision.

### Evidence that the two sources describe the same robot

Compared entity-by-entity, not assumed from names or joint counts:

* all **37** named bodies, joints and sites are present in both, with identical `pos`, `quat`,
  `axis` and `range`;
* all **14** inertials are identical;
* compiled total mass is **0.744361 kg** in both;
* the deployed `DEFAULT_POSITION` is byte-identical to the RL `HOME_FRAME` and to the source
  `scene.xml` `STAND` keyframe, and both sources document it the same way (the "STAND2" pose,
  trunk ~5 mm forward so the CoM sits over the ankle axis);
* the deployed daemon's firmware position gain (`Tuning::gain = 200`) matches the RL actuator
  configuration's `kp_fw = 200`, which is what makes the identified servo model below *the*
  matching one rather than *a* plausible one.

All nine deployed policy files were compared by SHA-256 against the bytes this repository
serves. They are identical. `tests/physics/microduck-phase5c-core.mjs` re-checks this.

## 2. Controller contract

Reproduced exactly from `duck-control/src/obs.rs`, `duck-control/src/model.rs`,
`duck-control/src/policy.rs` and `robotd/src/control.rs`.

```text
index   width  contents
0..3        3  gyro, trunk frame, rad/s
3..6        3  projected gravity, trunk frame, unit vector
6..20      14  joint position minus home pose, mouth excluded
20..34     14  joint velocity, mouth excluded
34..48     14  previous RAW policy action, before scaling, mouth excluded
48..61     13  command

48..51      3  vx, vy, vyaw
51..55      4  neck_pitch, head_pitch, head_yaw, head_roll
55..57      2  body x, y   - always zero, unbound in training
57          1  body z
58          1  body roll
59          1  body pitch
60          1  body yaw    - always zero, unbound in training
```

| Parameter | Value | Source |
|---|---|---|
| Observation / action width | 61 / 14 | `obs.rs` |
| Mouth wire index | 9, excluded from the policy on both sides | `model.rs` |
| Previous action | raw policy output, before scaling, shared across nets | `control.rs` |
| Observation history | none beyond that single previous action | `obs.rs` |
| Physics timestep | 0.005 s | `microduck_rl scripts/infer_policy.py` |
| Control decimation | 4 (20 ms, 50 Hz) | `infer_policy.py`, `robotd/src/main.rs` |
| Action scale, walking | 0.9 | `control.rs Tuning::action_scale` |
| Action scale, standing / kick / rise / ground pick / roulade | 1.0 | `control.rs` |
| Target low-pass | head joints 0.5, other joints 0.7 | `control.rs Tuning` |
| Standing threshold | 0.05 | `policy.rs` |
| Skill priority | roulade > kick > ground pick > sit/rise > stand-by-magnitude > walk | `control.rs` |

### Audited divergences between the two sources

Recorded rather than silently resolved:

* **Action scale.** Training used `JointPositionAction` scale 1.0 and the RL reference runner
  defaults to 1.0; the deployed daemon de-rates walking to 0.9. This repository pins the
  deployed runtime, so it reproduces 0.9. Measured effect in the native reference: about 7%
  less distance over 6 s.
* **Target filters.** The deployed source states the alpha policies are trained with
  head 0.5 / legs 0.7; the RL reference runner applies no filter and the training tree
  reverted its low-pass experiment. This repository reproduces the deployed filters. Measured
  effect: under 4% of walking distance either way.
* **Contact parameters.** PPO training additionally applies mjlab `FULL_COLLISION`, which sets
  `condim` 1 on non-foot colliders and gives the feet contact priority. The upstream CPU
  reference runner - the path that actually executes a deployed ONNX policy - compiles the raw
  scene, so that is what this package reproduces.
* **Actuator model.** This is the largest gap; see below.

## 3. Physical package

Three registered packages, all generated by `scripts/generate_microduck_models.py`
(`--check` regenerates and requires byte-identical output):

| Package | Asset | Purpose |
|---|---|---|
| `microduck-walk-519142b-v1` | `models/microduck/walk.xml` | the workspace |
| `microduck-walk-lowtraction-519142b-v1` | `models/microduck/walk_lowtraction.xml` | declared reduced-traction adverse fixture |
| `microduck-kick-519142b-v1` | `models/microduck/kick.xml` | walk plus the source ball prop |

* **Hierarchy, transforms, joint axes, ranges, inertials, named sites** - read verbatim from
  `assets/microduck/kinematics/robot_walk.xml`, the Apache-2.0 source model this repository
  already serves. Source-derived.
* **Servo** - the identified `chosen_actuator` model from `microduck_rl joints_properties.xml`:
  MuJoCo position actuator `kp` 0.55 N·m/rad at firmware gain 200, `kv` 0, force range
  ±0.96 N·m, control range ±10 rad, with joint damping 0.053, friction loss 0.0048 and
  armature 0.0018. Source-derived.
* **Physics** - Euler, 100 solver iterations, 50 line-search iterations, pyramidal cone,
  gravity −9.81, `condim` 3, friction (1.0, 0.005, 0.0001), `solref` (0.02, 1). Source-derived
  from the compiled reference scene.
* **Reset state** - trunk at z = 0.12 with identity orientation, joints and actuator targets at
  the home pose. The source keyframe spawns the robot 2.84 mm clear of the floor and lets it
  settle; this package keeps that exactly.
* **Ball** - the source prop: radius 0.035 m, mass 0.015 kg, thin-hollow-sphere inertia
  1.225e-5 kg·m², friction (0.5, 0.005, 0.0001), placed at the source kick-task offset
  (0.09, −0.042) m in the robot yaw frame. Training added ±15 mm of placement noise; this
  package does not.

### Collision geometry: what is source-derived and what is not

The identity, parent body, placement and contact mask of all **15** colliders are read from
`microduck_rl robot_allcollisions.xml`. Their **shape** is not.

The upstream colliders are CC BY-SA-NC mesh assets, which this repository does not
redistribute (the same exclusion the demonstrator already applies). Each is replaced by a
repository-authored box fitted to the corresponding source mesh envelope, measured from the
compiled upstream model. A box is the closer primitive for most of these parts: over half of
their vertices lie outside the inscribed ellipsoid of their own bounding box.

The two soles are the only load-bearing walking contact and are fitted more carefully: to the
**measured sole contact face** - the vertices within 1 mm of the sole plane, 45.6 × 34.0 mm -
rather than to the mesh bounding box, 54.0 × 41.2 mm. Fitting the bounding box would have
enlarged the support polygon by about 45% without saying so. Each sole box carries a 4.75°
roll about the ankle x axis, which is the plane the source sole face actually lies in.

Fit quality, measured at the home pose against the compiled upstream mesh model:

| Quantity | Upstream mesh | This package |
|---|---|---|
| Lowest sole point, world z | 0.00284 m | 0.00277 m |
| Sole contact patch, world x | −0.0158 … 0.0298 | −0.0158 … 0.0298 |
| Sole contact patch, world y (left) | 0.0249 … 0.0589 | 0.0247 … 0.0586 |
| Settled standing trunk height | 0.1161 m | 0.1161 m |

### Declared actuator-fidelity gap

The policies were trained against `microduck_rl`'s BAM M6 voltage-domain actuator, with a 3-6
control-tick action delay, per-environment battery voltage, load-dependent voltage sag and
joint-friction randomisation. **None of that is reproduced here.** What is reproduced is the
identified position model the upstream CPU reference runner executes, which is also what a
browser MuJoCo build can run. Observations are published as clean simulator ground truth, so
no robustness margin is measured. This is the largest declared fidelity gap in the package.

## 4. Capability audit

Every capability the MicroDuck product exposes, classified individually. Execution backend,
capability and evidence stay separate: the workspace being physical does not make every skill
in it physical.

| Capability | Status | Evidence |
|---|---|---|
| Standing | physical / verified | 6 s: trunk holds 0.1161 m at 0.22° tilt, both soles in contact 99.6% of physics steps, 0.0024 m drift |
| Walking / velocity control | physical / verified | 6 s at vx 0.30 m/s: 0.694 m along the commanded axis (0.119 m/s), 32/33 foot-contact transitions, 0.45 air fraction per foot |
| Sit / stand | physical / verified | trunk descends 0.1161 → 0.062 m under contact and holds; the rise returns it to standing |
| Ground pick | physical / **experimental** | real contact-driven crouch to 0.084 m and return; the scene carries no object to pick and the pinned sources define no pick-success criterion, so the motion is verified while the task is not |
| Left-leg kick | physical / verified | 7 named left-sole/ball contacts move the source 15 g ball 2.283 m |
| Right-leg kick | physical / verified | 5 named right-sole/ball contacts, 2.66 N peak normal force, ball 2.514 m |
| Fall recovery | physical / verified | from a declared face-down or face-up drop the standing policy returns the trunk to 0.1161 m below 0.25° tilt, through contact alone |
| Roulade | physical / **experimental** | real 0.511 m contact-driven forward roll peaking at 0.190 m and returning upright; a roll loads the trunk and head colliders hardest, and those are the fitted primitives |
| Roller-mode locomotion | **unsupported in physical mode** | no matched roller plant exists at the pinned revision |
| Roller crouch | **unsupported in physical mode** | same missing roller plant |

Roller and roller crouch are absent from the physical policy set by construction. The
controller cannot select a policy it does not carry, the compatibility gate refuses them, and
the WebMCP tool returns `CAPABILITY_UNSUPPORTED`. They are not routed to the legacy
demonstrator.

## 5. Physical evidence

All from `native/microduck_reference.py`, 6 s unless stated, deployed tuning.

### Nominal

| Trial | Commanded-axis distance | Speed | Trunk z | Tilt | Foot contact transitions |
|---|---|---|---|---|---|
| walk, vx 0.30 | +0.694 m | 0.119 m/s | 0.1163 | 2.06° | 32 / 33 |
| walk, vx 0.40 | +0.887 m | 0.153 m/s | 0.1167 | 1.95° | 32 / 33 |
| walk, vx 0.30 + vyaw 0.8 | +0.555 m | 0.111 m/s | 0.1167 | 1.13° | 31 / 32 |
| stand | n/a | 0.0004 m/s | 0.1161 | 0.22° | 2 / 2 |

Below about 0.30 m/s the matched policy holds a stand rather than initiating a gait. That is a
measured property of this policy in this plant, not a limit imposed by the workspace.

### Negative controls

| Control | Result |
|---|---|
| Actuation disabled (declared torque-off) | commanded-axis 0.123 m of *falling*, trunk collapses to 0.037 m at 67.8° tilt. No commanded locomotion mechanism remains. |
| Traction reduced (sole/floor friction 0.02) | commanded-axis distance falls from 0.694 m to 0.158 m, a **77% loss**; the robot slides 0.217 m sideways instead of walking |
| Kick miss (ball outside reach) | **0** foot-ball contacts, **0.0000 m** of ball motion. A miss stays a miss. |
| Recovery, actuation disabled | stays down at 0.0475 m / 89.9° |
| Recovery, reduced traction | thrashes for 8 s (190 contact transitions) and stays down at 0.0926 m / 85.7° |
| Recovery, 1.5 s budget | incomplete, reported as not recovered at 0.0771 m / 100.6° |
| Policy / model mismatch | rejected before execution, on any of the 14 compatibility-identity fields |

Recovery can fail, and does. No reset is ever counted as a recovery.

### Native / browser conformance

The controller-conformance fixture at `assets/microduck/fixtures/controller-conformance.json`
is generated by the native reference runner **independently of the browser implementation**,
from the pinned Rust controller sources. At six sampled controller states it records the
physical inputs, the assembled 61-value observation, the raw 14-value policy output, the
action scale, and the post-processed targets, plus the controller state carried into the next
tick. `tests/physics/microduck-phase5c-core.mjs` rebuilds each observation and each target set
from those inputs and compares value-for-value at 1e-6. A one-slot action shift - the shape an
off-by-one mouth mapping takes - is separately checked to produce different targets, so the
fixture demonstrably can catch it.

### Numerical sensitivity

Tolerance declared before the study: task-level conclusions must not change under a tighter
timestep, and commanded-axis walking distance must agree within ±10%.

| Physics timestep | Walking commanded-axis | Contact transitions | Standing trunk z |
|---|---|---|---|
| 5 ms (5× decimation 4) | +0.6943 m | 32 / 33 | 0.1161 m |
| 2.5 ms | +0.6903 m | 32 / 33 | 0.1161 m |
| 1 ms | +0.6890 m | 32 / 31 | 0.1161 m |

A **5× tighter timestep changes walking distance by 0.76%** and standing height not at all.

### Model-fit conformance against the upstream mesh model

The same deployed controller run against the upstream mesh-collider reference model and
against this package's fitted-primitive model:

| Tuning | Upstream mesh | This package | Delta |
|---|---|---|---|
| scale 1.0, no filter | 0.7829 m | 0.7341 m | −6.2% |
| scale 0.9, no filter | 0.7320 m | 0.6753 m | −7.7% |
| scale 1.0, filters | 0.8104 m | 0.7642 m | −5.7% |
| scale 0.9, filters (deployed) | 0.7425 m | 0.6932 m | −6.6% |

Across commanded velocities 0.30-0.50 m/s at the deployed tuning the agreement runs −10.1% to
+7.0%, with matching foot air fractions (0.44-0.48 in both).

**Two divergences from the upstream mesh model are recorded rather than smoothed over:**

1. **Gait onset at vx 0.25 m/s.** The upstream mesh model initiates a gait; this package does
   not. Both walk from 0.30 m/s upward. This is the boundary of the policy's gait-initiation
   behaviour, and the fitted soles land on the other side of it.
2. **Recovery from the on-side orientation.** The upstream mesh model **fails** (ends at
   0.0626 m / 95.9°); this package **succeeds** (0.1161 m / 0.22°). The settled initial
   conditions agree closely (0.0642 vs 0.0670 m, 101.0° vs 99.2°), so this is a genuine
   sensitivity at a marginal boundary, not a mapping error. Face-down and face-up recovery
   agree in both models, which is why recovery is claimed for those and this case is recorded
   here instead.

### Session and lifecycle

`tests/physics/microduck-lifecycle-core.mjs` drives the real `PhysicsSession`,
`BrowserMuJoCoBackend` and `MicroDuckController` through a scripted stand-in for the MuJoCo
worker. It checks the parts the browser lane would otherwise be the only witness to: the
declared setup path and its allowlist, the command budget, exactly `decimation` physics steps
per controller tick, ordinary control writing actuator targets and nothing else, a torque-off
condition removing the actuator command while the policy keeps producing targets, pause
freezing simulated time, cancellation, stale-epoch and foreign-session rejection, reset
clearing the controller feedback state, every published sample reaching the evaluator, and
dispose releasing the worker. It also drives `MicroDuckPhysicalSimulator` itself - the class
the workspace actually runs - through two injection seams, covering its control-loop cadence,
its bounded advance, cancellation mid-run leaving no further actuator writes, unsupported
capabilities reaching nothing, a declared perturbation clearing the controller feedback, and
the three state views staying separate.

### Browser lane: not run here

`tests/microduck-phase5c-browser.spec.mjs` is committed and registered in the Playwright
config. It could **not** be executed in the environment this work was done in: the sandbox's
egress proxy denies `cdn.jsdelivr.net`, which the application's import map uses for three.js
and Pyodide, so the app never reaches "Ready". This is not specific to the MicroDuck spec -
the pre-existing `tests/browser-smoke.spec.mjs` fails identically in the same environment. The
MicroDuck browser journey therefore remains **unrun**, and the `microduck-lifecycle-core`
suite above was written to cover as much of that ground as is reachable without a browser.

The same environment also blocks `tests/validate_task_patch.mjs`, which fetches a pinned task
from `jivishov/RoboBuddy_AI` over the network (HTTP 403). That failure is likewise
pre-existing and unrelated to Phase 5C; it reproduces with the Phase 5C task-catalog change
reverted.

## 6. What is not claimed

* **No hardware validation.** No measurement of an assembled MicroDuck was used anywhere in
  Phase 5C. Walking speed, traction, stability margin, kick distance, recovery probability and
  actuator dynamics are all unvalidated against hardware. Native and browser agreement is
  implementation conformance, not hardware accuracy.
* **No BAM actuator, delay buffer, or domain randomisation** - see the declared gap above.
* **No roller plant.**
* **No measured contact parameters.** Friction, contact stiffness and the floor are the
  declared parameters of the upstream reference scene on a flat indoor plane. Carpet, slopes,
  thresholds, rough terrain, foot wear, battery droop and servo thermal behaviour are outside
  the modelled scope.
* **No observation noise or delay**, so no robustness margin is measured.
* **No source ball-placement randomisation**, so kick robustness to placement error is not
  measured.

## 7. Licences

* MicroDuck-derived hierarchy, inertials, frames, policy bytes and visual: **Apache-2.0**
  (`licenses/microduck-apache-2.0.txt`).
* `microduck_rl` code and MJCF: **Apache-2.0**. Its **3D model files are CC BY-SA-NC**, so no
  STL or MJCF byte from it is redistributed by this repository; the reconciled scalar
  parameters and the fitted collider dimensions are recorded here with their derivation.
* Repository-authored colliders, floor, scenes and fixtures: **MIT**, except the generated
  evidence fixtures, which follow the repository's existing PolyForm-Noncommercial-1.0.0
  classification for recorded generated evidence.

## 8. Reproducing the evidence

```bash
python scripts/generate_microduck_models.py --check     # models are byte-reproducible
node   tests/physics/microduck-phase5c-core.mjs         # 30 contract/conformance gates
node   tests/physics/microduck-lifecycle-core.mjs       # 19 session/lifecycle gates
python native/microduck_reference.py --trial all        # every physical trial and negative control
python native/microduck_reference.py --trial walk --timestep 0.001   # numerical sensitivity
python native/microduck_reference.py --fixture assets/microduck/fixtures/controller-conformance.json
```

The reference runner needs `mujoco==3.11.0` (the pinned engine, `native/pyproject.toml`),
`numpy` and `onnxruntime`.
