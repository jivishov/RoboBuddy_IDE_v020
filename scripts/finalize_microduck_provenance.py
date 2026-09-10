#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one target, found {count}")
    return text.replace(old, new, 1)


def main() -> None:
    audit_path = ROOT / 'src/physics/microduck-source-audit.js'
    audit = audit_path.read_text()
    audit = audit.replace(
        "//     set, the identified servo model, the physics timestep, the control decimation, the\n//     reset pose and the ball prop - none of which exist in the deployed runtime source.",
        "//     task-specific collision plants, the BAM training-plant configuration, the source XML\n//     fallback actuator, the physics timestep, reset pose and ball prop - none of which exist\n//     in the deployed runtime source.",
    )
    if "'src/mjlab_microduck/robot/microduck/config_mjcf_walk.json'," not in audit:
        marker = "    'src/mjlab_microduck/robot/microduck/robot_allcollisions.xml',\n"
        audit = replace_once(
            audit,
            marker,
            marker + "    'src/mjlab_microduck/robot/microduck/config_mjcf_walk.json',\n",
            'RL source path insertion',
        )

    actuator_collision_block = """  row('firmware gain schedule', '200 running; 160 standing / stand-tuned skills', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Tuning::gain / standing_gain_ratio',
    'The physical worker passes the scheduled firmware gain into the BAM voltage-domain torque law. It does not emulate the register by mutating a MuJoCo position-actuator stiffness.'),
  row('BAM actuator model', 'Rhoban/bam v1.0.1 XL330/M6 voltage-domain torque motor', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'Rhoban/bam@ab81512c44f1f709b99ef332addb5e51568cd51c + microduck_rl actuator configuration',
    'Browser equations are independently cross-checked against better-actuator-models==1.0.1. Motor torque depends on target error, firmware gain, available voltage and back-EMF; MuJoCo ctrl is motor torque, not a position target.'),
  row('BAM armature and friction', 'identified armature 0.0018077432831600838 kg m^2 plus BAM directional/Stribeck/load/viscous friction', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'Rhoban/bam v1.0.1 M6 parameters',
    'BAM replaces source joint damping/frictionloss for the physical plant and supplies its own dynamic friction and identified armature. Training-reference friction scaling applies to the velocity-independent BAM friction budget; viscous friction remains nominal.'),
  row('source XML fallback actuator', 'position kp 0.55 N m/rad, kv 0, force range +/-0.96 N m, control range +/-10 rad', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl joints_properties.xml class chosen_actuator',
    'Retained only as source-XML/fallback evidence and pre-conversion validation. It is not the browser or native Phase 5C physical actuator authority.'),
  row('collision plant routing', 'walk/lowTraction -> robot_walk.xml; stand/recovery/sit_stand/ground_pick/roulade -> robot_allcollisions.xml; kick -> robot_allcollisions.xml + ball.xml', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl robot_walk.xml, robot_allcollisions.xml and pinned task configs',
    'Walking deliberately uses the reduced source collision plant. Body-on-ground skills and kick use the broad all-collision source plant. The pinned revision does not contain the roulade task configuration, so roulade uses the broad plant conservatively and remains physical/experimental.'),
  row('collision mesh bytes', 'exact pinned upstream STL bytes, no fitted primitive replacement', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl@519142b1f5bf59fdfd44d06c205119e7fff8e3cb assets + models/microduck/source/GIT_BLOB_SHA1SUMS',
    'The committed STL files are byte-compared against the pinned checkout and their Git blob identities are recorded. Generated task wrappers preserve source mesh geometry.'),
  row('contact parameters', 'task-specific source masks/priority/friction with explicit low-traction negative fixture', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl config_mjcf_walk.json / robot XML / ball.xml',
    'The normal task plants preserve source contact semantics. The low-traction package is the separately declared adverse fixture; it is not presented as a measured material.'),
  row('floor', 'flat plane at z = 0', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl scene/reference environment', 'A declared flat indoor-floor reference, not a measurement of a particular physical surface.'),
  row('reduced-traction surface'"""
    audit, n = re.subn(
        r"  row\('firmware position gain'.*?\n  row\('reduced-traction surface'",
        actuator_collision_block,
        audit,
        count=1,
        flags=re.S,
    )
    if n != 1:
        raise RuntimeError(f'stale actuator/collision audit block: expected one, found {n}')

    observation_block = """  row('deployment IMU preprocessing', 'normalised projected gravity plus per-axis median-of-three on gyro and projected gravity', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'pollen-robotics/microduck@590b986 duck-control/src/imu.rs',
    'The deployment-reference workspace reproduces the deployed median-of-three pipeline; its history resets to two zero gyro samples and two upright gravity samples.'),
  row('deployment target delay', 'no synthetic target delay injected', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'deployed runtime control path',
    'The deterministic deployment-reference keeps the deployed controller path separate from training-only domain randomisation.'),
  row('training-reference randomisation', 'Vin 6.5..8.2 V; sag 0..0.2 V/Nm; 3..6 physics-step target delay; friction/armature 0.9..1.1; mass/inertia 0.95..1.05; encoder bias +/-0.015 rad; IMU mount +/-6 deg', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl training configuration and BAM integration',
    'These values are retained as a separate training-reference profile. They are not silently injected into the interactive deployment-reference workspace.'),
  row('deployment electrical reference', '7.4 V nominal, 0.1 V/Nm sag coefficient, 6.0 V minimum', PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
    'source CPU regression/reference condition',
    'A deterministic source-backed rehearsal condition, not a battery/internal-resistance measurement from a particular assembled MicroDuck.'),
  row('hardware alignment'"""
    audit, n = re.subn(
        r"  row\('observation noise and delay'.*?\n  row\('hardware alignment'",
        observation_block,
        audit,
        count=1,
        flags=re.S,
    )
    if n != 1:
        raise RuntimeError(f'stale observation audit block: expected one, found {n}')

    old_required = """    'physics timestep', 'action scaling (walk)', 'target low-pass filters', 'servo model',
    'joint damping / frictionloss / armature', 'collision set', 'collider primitive fit',
    'sole contact face', 'contact parameters', 'reset state', 'hardware alignment',"""
    new_required = """    'physics timestep', 'action scaling (walk)', 'target low-pass filters', 'firmware gain schedule',
    'BAM actuator model', 'BAM armature and friction', 'source XML fallback actuator',
    'collision plant routing', 'collision mesh bytes', 'contact parameters', 'reset state',
    'deployment IMU preprocessing', 'training-reference randomisation', 'deployment electrical reference', 'hardware alignment',"""
    audit = replace_once(audit, old_required, new_required, 'reconciliation required list')
    audit_path.write_text(audit)

    cap_path = ROOT / 'src/physics/microduck-capabilities.js'
    capabilities = cap_path.read_text()
    new_capability_audit = """export const MICRODUCK_CAPABILITY_AUDIT = Object.freeze([
  Object.freeze({
    id: 'stand', label: 'Standing', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'stand',
    evidence: 'Independent BAM/MuJoCo evidence requires a stable stance on the broad source collision plant with real sole contacts and finite actuator force; no root-state write is used to hold the pose.',
  }),
  Object.freeze({
    id: 'walk', label: 'Walking / velocity control', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'walking',
    evidence: 'Independent BAM/MuJoCo evidence requires commanded-axis locomotion with real foot-contact transitions. Torque-off must remove propulsion and the reduced-traction fixture must materially degrade locomotion; timestep sensitivity is checked separately.',
  }),
  Object.freeze({
    id: 'sit_stand', label: 'Sit / stand', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'sitstand',
    evidence: 'Independent BAM/MuJoCo evidence runs this policy only on the broad source collision plant and verifies the posture transition from simulated state/contact rather than a scripted root pose.',
  }),
  Object.freeze({
    id: 'ground_pick', label: 'Ground pick', status: S.PHYSICAL_EXPERIMENTAL, physicalPolicy: 'ground_pick',
    evidence: 'The policy runs on the broad source collision plant and its physical crouch/return is contact-driven. The scene carries no object to pick up and the pinned sources provide no pick-success criterion, so the motion is physical while the task remains experimental.',
  }),
  Object.freeze({
    id: 'kick_left', label: 'Left-leg ball kick', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'kick_left',
    evidence: 'Independent BAM/MuJoCo evidence requires named left-foot/ball contact before ball motion can count as a kick. An out-of-reach miss control must produce no qualifying contact and no credited kick.',
  }),
  Object.freeze({
    id: 'kick_right', label: 'Right-leg ball kick', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'kick_right',
    evidence: 'Independent BAM/MuJoCo evidence requires named right-foot/ball contact before ball motion can count as a kick. An out-of-reach miss control must produce no qualifying contact and no credited kick.',
  }),
  Object.freeze({
    id: 'recovery', label: 'Fall recovery', status: S.PHYSICAL_VERIFIED, physicalPolicy: 'stand',
    evidence: 'Runs only on the source all-collision ground-contact plant. Positive and negative BAM/MuJoCo recovery controls start from declared setup orientations and are judged from actuator force, trunk pose and contact; reset is never counted as recovery.',
  }),
  Object.freeze({
    id: 'roulade', label: 'Roulade (forward roll)', status: S.PHYSICAL_EXPERIMENTAL, physicalPolicy: 'roulade',
    evidence: 'The deployed runtime includes roulade.onnx and it is conservatively routed to the broad exact-mesh source collision plant. The pinned microduck_rl revision does not contain the roulade task configuration, so no exact pinned training-task collision-match claim is made.',
  }),
  Object.freeze({
    id: 'roller', label: 'Roller-mode locomotion', status: S.UNSUPPORTED, physicalPolicy: null,
    evidence: 'No matched physical roller plant exists at the pinned RL revision. Running the roller policy on the walking or broad ground-contact plant would be a policy/model mismatch, so it has no physical route.',
  }),
  Object.freeze({
    id: 'roller_crouch', label: 'Roller crouch', status: S.UNSUPPORTED, physicalPolicy: null,
    evidence: 'Same missing matched roller plant as roller-mode locomotion; it has no physical route.',
  }),
]);

export const MICRODUCK_PHYSICAL_POLICIES"""
    capabilities, n = re.subn(
        r"export const MICRODUCK_CAPABILITY_AUDIT = Object\.freeze\(\[.*?\]\);\n\nexport const MICRODUCK_PHYSICAL_POLICIES",
        new_capability_audit,
        capabilities,
        count=1,
        flags=re.S,
    )
    if n != 1:
        raise RuntimeError(f'capability evidence block: expected one, found {n}')
    cap_path.write_text(capabilities)

    provenance = """# MicroDuck Phase 5C physical provenance

This document records the **physical MuJoCo MicroDuck workspace** after the full-plant fidelity migration. It is separate from the legacy policy demonstrator, which is explicitly selectable and is never used as a fallback when the physical backend fails.

## 1. Authority and pinned sources

| Role | Source | Pin |
|---|---|---|
| Deployed controller/runtime and policy bytes | `pollen-robotics/microduck` | `590b986bd8c0d50ae02cb3ea2f59c463b6828168` |
| Physical/RL environment and task collision models | `pollen-robotics/microduck_rl` | `519142b1f5bf59fdfd44d06c205119e7fff8e3cb` |
| BAM actuator equations/identified M6 parameters | `Rhoban/bam` | v1.0.1, `ab81512c44f1f709b99ef332addb5e51568cd51c` |

The deployed source is authoritative for the 61-value observation layout, 14 policy joints, home pose, command encoding, raw previous-action feedback, action scaling/filtering, skill priority, standing threshold, 50 Hz controller cadence, firmware-gain schedule and deployed IMU preprocessing. The RL source is authoritative for the physical robot model, task-specific collision plants, reset pose, ball prop, training-domain randomisation and MicroDuck BAM integration. BAM v1.0.1 is the actuator-equation authority.

## 2. Task-specific collision plants

MicroDuck does **not** use one universal collision approximation. Four registered packages preserve the task distinction in the pinned environment:

| Package key | Model | Physical use |
|---|---|---|
| `walk` | `models/microduck/walk.xml` | normal locomotion; reduced upstream `robot_walk.xml` collision set |
| `lowTraction` | `models/microduck/walk_lowtraction.xml` | declared low-friction locomotion negative-control fixture |
| `groundContact` | `models/microduck/groundcontact.xml` | stand, recovery, sit/stand, ground-pick and conservative roulade mapping; upstream `robot_allcollisions.xml` |
| `kick` | `models/microduck/kick.xml` | all-collision robot plus the source `ball.xml` prop |

A capability requested on the wrong collision plant is rejected with `wrong-plant` before it reaches the physical authority. Roller and roller-crouch have no matched plant at this pin and remain unsupported in physical mode.

The pinned `microduck_rl` revision does not contain the roulade task configuration even though the deployed runtime contains `roulade.onnx`. Roulade is therefore mapped conservatively to the broad all-collision plant and remains **physical / experimental**, not an exact pinned training-task match.

## 3. Collision geometry provenance

The physical packages use the **exact STL collision mesh bytes** from the pinned `microduck_rl` checkout. They are not fitted boxes, ellipsoids or other repository-authored primitive replacements. Source MJCF is retained under `models/microduck/source/`; exact STL bytes under `models/microduck/assets/`; `models/microduck/source/GIT_BLOB_SHA1SUMS` records each imported mesh's Git blob identity. CI compares committed XML/STL bytes against a fresh checkout of the exact pin and checks deterministic regeneration of the task wrappers.

`walk_lowtraction.xml` is intentionally different: sole/floor sliding friction is reduced to 0.02 as an adverse-condition fixture. It is not represented as a measured physical material.

## 4. BAM actuator plant

The source XML position actuator (`kp=0.55`, `kv=0`, `forcerange=±0.96 N·m`) is retained only as fallback/source-model evidence. It is **not** the Phase 5C physical actuator authority.

Browser and native execution use the BAM v1.0.1 XL330/M6 voltage-domain torque model. Requested joint targets stay separate from MuJoCo `ctrl`; `ctrl` is motor torque. BAM supplies voltage/back-EMF behavior, identified armature, directional/Stribeck/load-dependent friction, viscous friction and load-dependent voltage sag. JavaScript equations are independently cross-checked against `better-actuator-models==1.0.1` in CI.

The deployed firmware gain remains part of the physical law: 200 while running and 160 for standing/stand-tuned behavior. Torque-off removes electromagnetic motor torque without overwriting the requested target.

## 5. Deployment-reference observation/control path

Physics timestep is 5 ms and the policy/control interval is 20 ms (50 Hz). The policy receives the deployed 61-value contract: `3 gyro + 3 projected gravity + 14 (q-home) + 14 qdot + 14 previous raw action + 13 command`.

The mouth is excluded from the 14 policy joints. Previous action is the **raw policy output before scaling**. The standing threshold is inclusive (`<= 0.05`). Projected gravity is normalized, then the interactive physical workspace applies the deployed per-axis median-of-three preprocessing to gyro and projected gravity.

## 6. Training-reference and deployment-reference remain separate

The encoded training-reference preserves the pinned training distributions: firmware gain 200; battery voltage 6.5–8.2 V; voltage-sag coefficient 0–0.2 V/Nm with 6.0 V minimum; target delay 3–6 **physics steps**; friction and armature scaling 0.9–1.1; mass/inertia scaling 0.95–1.05; encoder bias ±0.015 rad; and IMU mount randomisation ±6°.

The interactive deployment-reference uses no synthetic training delay or stochastic domain randomisation. Its deterministic 7.4 V and 0.1 V/Nm sag settings are a source-backed CPU regression/reference condition for reproducible rehearsal. They are **not measurements of an assembled robot battery or internal resistance**.

## 7. Physical-authority invariants

MuJoCo is the single state authority. Outside explicit logged setup/reset operations, the physical path does not write root pose, root velocity, joint state or object state directly. There is no synthetic kick impulse, boundary clamp, teleport, hidden weld/grasp, scripted recovery or success-state overwrite.

Locomotion evidence requires real foot contacts. A kick can be credited only after a named kicking-foot/ball contact. Recovery is judged from simulated pose/contact/actuator evidence after a declared initial orientation; reset is never recovery. Live-Python and WebMCP expose bounded commands and skills, not arbitrary joint/root state writes.

## 8. Verification and evidence limits

Permanent gates cover deterministic model regeneration, asset hashes, exact pinned source XML/STL identity, controller/observation compatibility, collision-plant routing, BAM equation conformance, native MuJoCo/BAM positive and negative controls, timestep sensitivity, fleet regressions and a browser physical journey.

This is **software/model fidelity**, not assembled-hardware calibration. No repository evidence establishes exact physical walking speed, recovery probability, battery/internal resistance, real bus latency, thermal behavior, individual servo variation, wear or real contact-material coefficients for a particular MicroDuck. Those remain hardware-validation items.

## 9. Licensing

RoboBuddy original software remains MIT-licensed under `LICENSES/MIT.txt`. Applicable upstream material retains its upstream terms. The exact MicroDuck 3D model/collision assets are outside the RoboBuddy MIT grant; upstream currently declares its 3D model files **Creative Commons BY-SA-NC** and does not state a Creative Commons version. This repository does not invent one. See `LICENSE` and `THIRD_PARTY_NOTICES.md` for component scope.
"""
    (ROOT / 'docs/physics/microduck-provenance.md').write_text(provenance)

    stale_patterns = [
        r'repository-authored boxes',
        r'does not redistribute',
        r'largest declared actuator-fidelity gap',
        r'Neither is reproduced',
    ]
    combined = audit_path.read_text() + '\n' + (ROOT / 'docs/physics/microduck-provenance.md').read_text()
    for pattern in stale_patterns:
        if re.search(pattern, combined, re.I):
            raise RuntimeError(f'stale provenance statement remains: {pattern}')
    print('MicroDuck provenance reconciliation complete')


if __name__ == '__main__':
    main()
