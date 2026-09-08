# RoboBuddy IDE — Physics Preview

Version `0.2.0-alpha.2` completes the SO-101 physical migration milestone: the normal SO-101 workspace, live Python, renderer, task evaluator, and bounded WebMCP controls share one authoritative browser MuJoCo PhysicsSession.

Phase 5A also migrates the normal OpenArm V2 `openarm-04-filtration-workcell` task to a single-authority bimanual MuJoCo workspace on the isolated migration branch. It preserves the pinned V2 mirrored joint/actuator semantics, uses true free-body dry vessels, source-defined mechanical finger coupling only, simulation-time async Python, causal contact/support task evaluation, and bounded OpenArm WebMCP control. Primitive browser collision surrogates and the synthetic dry workcell remain simulator approximations; hardware calibration is pending.

The supported SO-101 task is a synthetic rigid-body block-transfer benchmark. Simulator validation is not hardware calibration. OpenArm, LeKiwi, MicroDuck, Unitree G1, Panda, and ASIMOV remain separate later robot-specific migration work.
