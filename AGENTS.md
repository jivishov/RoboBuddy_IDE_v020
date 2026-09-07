# RoboBuddy IDE v0.2.0 Physics Migration Guardrails

## Repository isolation

- `jivishov/RoboBuddy_IDE_v020` is the only write target for this migration.
- `jivishov/RoboBuddy_IDE` is read-only while the WebMCP challenge review is active.
- `jivishov/RoboBuddy_AI` is also read-only for this migration.
- Do not create branches, tags, issues, pull requests, deployments, releases, or settings changes in either read-only repository.

## Physics rules

- Physical mode must derive motion from declared dynamics, contacts, constraints, and controllers.
- Do not directly overwrite root pose/velocity, object velocity, or joint state to create successful task outcomes.
- Do not use hidden grasp welding, synthetic successful kicks, boundary clamping-as-physics, or reset-as-recovery.
- Setup/reset interventions must be explicit and logged.
- Rendering must not be the authoritative physical clock.
- WebMCP remains an opt-in bounded tool layer; it is not the physics transport.

## Evidence labels

Keep execution backend, capability, and evidence separate. Never claim hardware fidelity without named measurements and scope.
