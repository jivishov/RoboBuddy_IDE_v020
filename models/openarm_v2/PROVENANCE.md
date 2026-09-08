# OpenArm V2 Phase 5A physical model provenance

## Source authority

RoboBuddy Phase 5A uses the OpenArm **V2** MuJoCo model from:

- repository: `enactic/openarm_mujoco`
- pinned revision: `a8c979629f2591ad035d99d338ce114969e6cddc`
- source robot model: `v2/openarm_bimanual.xml`
- source cell relationship: `v2/cell/cell.xml`
- upstream license: Apache-2.0

The standard Apache-2.0 license text is vendored under `licenses/` for the OpenArm-derived model package.

The Phase 5A browser asset is `models/openarm_v2/phase5a.xml`, SHA-256
`5959055a559bdc2d68b494f678242d5b2d166b76483ede7008e9ea8ade238101`.

## Source-derived robot parameters

The browser adaptation preserves the pinned V2 source values for:

- the mirrored left/right seven-joint kinematic chains;
- joint axes and joint limits;
- rigid-body masses, centers of mass, inertial orientations, and diagonal inertias;
- DM8009, DM4340, DM4310, and DM3507 joint damping, armature, and friction-loss values;
- V2 position-actuator gains, control ranges, and force limits;
- the left/right two-finger mechanical equality couplings;
- elliptic contact-cone configuration and `impratio=10`;
- the V2 cell home mount relationship: arm root at cell `x=0.185 m`, `z=1.340 m` when the source lifter is at `q=0`;
- the source cell worktop top at `z=1.005 m`.

The V2 cell lifter is deliberately **fixed at its source home q=0 pose** for Phase 5A. It is not an exposed actuator in live Python or WebMCP and cannot move to make a task reachable.

## Explicit adaptations / estimates

The following are not claimed to be exact upstream or installed-hardware values:

- upstream robot mesh collision geometry is replaced by named primitive collision surrogates so the package is self-contained in the browser;
- empty-flask and empty-beaker collision envelopes, mass, inertia, and friction are controlled benchmark estimates;
- unpowered-hotplate and ring-stand/wire-gauze geometry is a controlled dry-workcell benchmark approximation;
- target regions and task-evaluator tolerances are simulator acceptance criteria, not hardware performance specifications.

The legacy `openarm-04-filtration-workcell` source-plant task remains a regression/task-objective reference. Its degree/mm kinematic state and `attachedTo` object semantics are **not** used by the Phase 5A physical execution path.

## Physical task semantics

The normal OpenArm task keeps the public task ID `openarm-04-filtration-workcell` but now runs as `physical_mujoco` through one authoritative `PhysicsSession`.

- Flask and beaker are MuJoCo free bodies.
- Ordinary task execution writes actuator controls only.
- No task weld, equality attachment, parent switch, snap, transform overwrite, or qpos/qvel write is available during the task.
- The only equality constraints are the source-derived left/right finger mechanical couplings.
- Task success requires observed finger contact, lift while held, held transport, intended support contact, release, stable post-release rest from pose/velocity evidence, retreat, left-then-right ordering, and both objects remaining supported in the final shared world state.
- Reset is deterministic initial-condition creation only and cannot count as task recovery.

## Claim boundary

This phase establishes browser/native MuJoCo simulator evidence for the declared model and dry benchmark. It does **not** establish:

- hardware trajectory accuracy, backlash, compliance, CAN latency, or motor thermal behavior;
- exact upstream mesh-collision parity;
- physical payload certification or glass safety;
- force/tactile sensing;
- liquids, heating, temperature, filtration, or other process physics.
