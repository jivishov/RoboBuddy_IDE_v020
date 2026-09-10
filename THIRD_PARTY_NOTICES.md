# Third-party notices

## MicroDuck runtime and controller sources

Source: `pollen-robotics/microduck` pinned by the MicroDuck source audit. Upstream software license: Apache-2.0. RoboBuddy uses source-derived hierarchy/controller parameters and exact policy/runtime artifacts as documented in `assets/microduck/manifest.json` and `docs/physics/microduck-provenance.md`.

## MicroDuck RL physical model and collision assets

Source: `pollen-robotics/microduck_rl` at `519142b1f5bf59fdfd44d06c205119e7fff8e3cb`.

Upstream's current project license notice states: the project software is Apache 2.0 and **3D model files are licensed under Creative Commons BY-SA-NC**. That notice does not name a Creative Commons version. This repository therefore preserves the upstream wording and intentionally does not substitute a version-specific CC license.

The exact source MJCF files used for physical collision semantics are retained in `models/microduck/source/`. Exact source STL bytes are retained in `models/microduck/assets/`. `models/microduck/source/GIT_BLOB_SHA1SUMS` records their Git blob identities against the pinned source checkout. Generated RoboBuddy wrappers `walk.xml`, `walk_lowtraction.xml`, `groundcontact.xml`, and `kick.xml` preserve source mesh geometry while adding only the local floor, deterministic task prop placement, actuator names, timestep, and keyframe needed by the browser/native harness.

Walking uses the pinned reduced `robot_walk.xml` collision set. Kick and recovery evidence use the pinned `robot_allcollisions.xml` plant, matching the task-specific upstream configuration rather than applying one collision approximation to all tasks.

## BAM

Source: `Rhoban/bam` v1.0.1, commit `ab81512c44f1f709b99ef332addb5e51568cd51c`. The MicroDuck plant reproduces the source XL330/M6 voltage-domain actuator equations and is independently checked against `better-actuator-models==1.0.1`. Refer to that upstream project for its license terms.

## Important scope note

The component licenses above are not relicensed by RoboBuddy's MIT default. This file records provenance and scope; it is not a claim that hardware calibration has been performed.
