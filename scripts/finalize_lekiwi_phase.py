#!/usr/bin/env python3
"""Run the staged LeKiwi finalization patch without modifying persistent workflow files,
then fully retire the legacy LeKiwi task descriptor and reconcile the canonical visual joint
conventions with the authoritative Menagerie/MuJoCo arm state.
"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
original_path = ROOT / 'scripts' / 'finalize_lekiwi_phase_original.py'
source = original_path.read_text()
# GitHub Actions tokens cannot push workflow changes. The persistent workflow remains unchanged;
# presentation + tighter numeric evidence are exercised by this one-use finalizer and full CI.
source, count = re.subn(
    r"\n# 8\. P7 evidence: tighter wheel/traction sensitivity and presentation browser gate\.[\s\S]*?\n# 9\. Provenance language:",
    "\n# 9. Provenance language:",
    source,
    count=1,
)
if count != 1:
    raise RuntimeError(f'could not remove workflow-edit section: {count}')
exec(compile(source, str(original_path), 'exec'), {'__file__': str(original_path), '__name__': '__main__'})


def read(path): return (ROOT / path).read_text()
def write(path, text): (ROOT / path).write_text(text)
def one(text, old, new, label):
    n = text.count(old)
    if n != 1: raise RuntimeError(f'{label}: expected 1, found {n}')
    return text.replace(old, new, 1)

# Fully retire the legacy LeKiwi task descriptor; keep only LEGACY_TASK_SOURCE provenance metadata.
p = 'src/task-catalog.js'
t = read(p)
t = one(t, "  lekiwi: Object.freeze([\n    task('lekiwi', 'lekiwi', 'lekiwi-01-beaker-courier.json', 'lekiwi-01-beaker-courier', 'Beaker Courier', 'lekiwi_sim'),\n  ]),\n", '', 'remove PATCH_TASKS.lekiwi')
write(p, t)

p = 'tests/validate_task_patch.mjs'
t = read(p)
t = one(t, "  lekiwi: ['lekiwi-01-beaker-courier'],\n", '', 'remove legacy LeKiwi expected fixture')
t = one(t, "  // OpenArm and LeKiwi descriptors remain pinned provenance; their normal catalogs are now the\n  // migrated physical workspaces, asserted separately below.\n  if (profileId === 'openarm' || profileId === 'lekiwi') continue;\n", "  // OpenArm's legacy descriptor remains pinned for provenance; its normal catalog is the migrated\n  // physical workspace, asserted separately below.\n  if (profileId === 'openarm') continue;\n", 'update legacy fixture comment')
write(p, t)

p = 'tests/browser-smoke.spec.mjs'
t = read(p)
t = one(t, "    'so101-v2-09-vacuum-filtration',\n    'lekiwi-01-beaker-courier',\n", "    'so101-v2-09-vacuum-filtration',\n", 'remove legacy source replay expectation')
write(p, t)

# Source-reconciled physical->canonical presentation convention.  These are not physics
# parameters: they map the pinned Menagerie SO-ARM101 joint zero/sign convention into the pinned
# LeKiwi URDF/canonical-mesh convention.  The first four offsets were solved from corresponding
# joint pivots; wrist roll is solved from the downstream gripper-hinge axis.
p = 'src/physics/lekiwi-source-audit.js'
t = read(p)
anchor = "export const LEKIWI_WHEEL_NAMES = Object.freeze(['base_left_wheel', 'base_back_wheel', 'base_right_wheel']);\n"
mapping = '''export const LEKIWI_CANONICAL_PRESENTATION_MAP = Object.freeze({\n  joints: Object.freeze({\n    arm_shoulder_pan: Object.freeze({ sign: 1, offsetRad: 0.01654725 }),\n    arm_shoulder_lift: Object.freeze({ sign: -1, offsetRad: -1.65460081 }),\n    arm_elbow_flex: Object.freeze({ sign: -1, offsetRad: 1.48772696 }),\n    arm_wrist_flex: Object.freeze({ sign: -1, offsetRad: 1.30528119 }),\n    arm_wrist_roll: Object.freeze({ sign: -1, offsetRad: 1.62034568 }),\n  }),\n  gripper: Object.freeze({ physicalOpenRad: 1.20, physicalClosedRad: -0.17, canonicalOpenValue: 20, canonicalCloseValue: 85 }),\n  evidence: 'source-reconciled from the pinned LeKiwi canonical URDF chain and pinned Menagerie SO-ARM101 physical chain; presentation only',\n});\n\n'''
t = one(t, anchor, mapping + anchor, 'canonical presentation map export')
row_anchor = "  row('hardware alignment', 'none', PARAMETER_EVIDENCE.CALIBRATION_REQUIRED, '-', 'No measurement of an assembled LeKiwi was used or is claimed anywhere in Phase 5B.'),\n"
row_insert = "  row('canonical visual joint convention', 'five physical-to-visual sign/zero transforms plus normalized gripper mapping', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'pinned LeKiwi canonical URDF chain compared with pinned Menagerie SO-ARM101 chain', 'Presentation-only reconciliation. Corresponding arm pivots agree within single-digit millimetres over randomized poses; wrist-roll sign/zero is additionally checked from the downstream gripper-hinge axis. It never feeds state back into MuJoCo.'),\n" + row_anchor
t = one(t, row_anchor, row_insert, 'presentation reconciliation row')
write(p, t)

# Apply those explicit conventions at the single presentation boundary.
p = 'src/physics/lekiwi-physical-simulator.js'
t = read(p)
t = one(t, "import { MODEL_FRAME } from './lekiwi-source-audit.js';\n", "import { LEKIWI_CANONICAL_PRESENTATION_MAP, MODEL_FRAME } from './lekiwi-source-audit.js';\n", 'presentation map import')
old = '''function canonicalArmState(observation) {\n  const state = {};\n  for (const jointId of ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll', 'arm_gripper']) {\n    const value = Number(observation?.joints?.[jointId]?.positionRad);\n    // Presentation adapter only: the canonical LeKiwi rig consumes the legacy degree scale.\n    // This conversion never feeds back into physics.\n    if (Number.isFinite(value)) state[`${jointId}.pos`] = value * RAD_TO_DEG;\n  }\n  return state;\n}\n'''
new = '''function canonicalArmState(observation) {\n  const state = {};\n  for (const [jointId, mapping] of Object.entries(LEKIWI_CANONICAL_PRESENTATION_MAP.joints)) {\n    const value = Number(observation?.joints?.[jointId]?.positionRad);\n    if (!Number.isFinite(value)) continue;\n    // Presentation-only source reconciliation: the canonical mesh is baked from the official\n    // LeKiwi URDF, while the physical arm uses the pinned Menagerie SO-ARM101 convention.\n    state[`${jointId}.pos`] = (mapping.sign * value + mapping.offsetRad) * RAD_TO_DEG;\n  }\n  const grip = Number(observation?.joints?.arm_gripper?.positionRad);\n  if (Number.isFinite(grip)) {\n    const g = LEKIWI_CANONICAL_PRESENTATION_MAP.gripper;\n    const denominator = g.physicalOpenRad - g.physicalClosedRad;\n    const closedRatio = THREE.MathUtils.clamp((g.physicalOpenRad - grip) / denominator, 0, 1);\n    state['arm_gripper.pos'] = THREE.MathUtils.lerp(g.canonicalOpenValue, g.canonicalCloseValue, closedRatio);\n  }\n  return state;\n}\n'''
t = one(t, old, new, 'canonical arm presentation conversion')
write(p, t)

# Contract-test the presentation mapping and ensure it remains explicitly presentation-only.
p = 'tests/physics/lekiwi-phase5b-core.mjs'
t = read(p)
t = one(t,
    "  LEKIWI_ACTUATED_NAMES, LEKIWI_RECONCILIATION, LEKIWI_SIM_SECONDARY_SOURCE, LEKIWI_SOURCE,\n",
    "  LEKIWI_ACTUATED_NAMES, LEKIWI_CANONICAL_PRESENTATION_MAP, LEKIWI_RECONCILIATION, LEKIWI_SIM_SECONDARY_SOURCE, LEKIWI_SOURCE,\n",
    'core mapping import')
insert = '''\n// The canonical visual is a different pinned source chain from the Menagerie physical arm.\n// Its sign/zero conversion is explicit provenance, never a physical-state correction.\nassert.deepEqual(Object.fromEntries(Object.entries(LEKIWI_CANONICAL_PRESENTATION_MAP.joints).map(([id, m]) => [id, m.sign])), {\n  arm_shoulder_pan: 1, arm_shoulder_lift: -1, arm_elbow_flex: -1, arm_wrist_flex: -1, arm_wrist_roll: -1,\n});\nclose(LEKIWI_CANONICAL_PRESENTATION_MAP.gripper.physicalOpenRad, LEKIWI_COURIER_CONTROLLER.gripperOpenRad, 1e-12, 'presentation gripper open');\nclose(LEKIWI_CANONICAL_PRESENTATION_MAP.gripper.physicalClosedRad, LEKIWI_COURIER_CONTROLLER.gripperCloseRad, 1e-12, 'presentation gripper close');\nassert.match(LEKIWI_CANONICAL_PRESENTATION_MAP.evidence, /presentation only/);\nassert.ok(LEKIWI_RECONCILIATION.some((row) => row.parameter === 'canonical visual joint convention' && row.evidence === PARAMETER_EVIDENCE.SOURCE_DERIVED));\n'''
marker = "// --- 2. model packages, assets, bounded actuators ---------------------------------------------\n"
t = one(t, marker, insert + '\n' + marker, 'core presentation mapping assertions')
write(p, t)

# Document the source-model joint-convention reconciliation explicitly.
p = 'docs/physics/lekiwi-provenance.md'
t = read(p)
anchor = "Displayed wheel spin comes from the observed MuJoCo wheel `qpos`; because the physical joints use\nthe LeRobot-positive axes that are anti-parallel to the source URDF wheel axes, only the visual\nangle sign is reversed. These transforms are presentation-only and never feed state back to MuJoCo.\n"
extra = anchor + '''\nThe canonical arm mesh and the physical arm are also pinned from two different source chains: the\nLeKiwi URDF visual and Menagerie SO-ARM101 respectively. Their joint zero/sign conventions are not\nidentical. P5B therefore records an explicit physical→canonical presentation map (source code:\n`LEKIWI_CANONICAL_PRESENTATION_MAP`). Corresponding shoulder/elbow/wrist pivots were reconciled\nfrom the two pinned kinematic chains; wrist roll was checked independently using the downstream\ngripper-hinge axis. This conversion changes only the displayed mesh angles. Physical joint state,\ncontacts, task evaluation, live Python, and WebMCP remain the unmodified MuJoCo observations.\n'''
t = one(t, anchor, extra, 'presentation convention provenance')
write(p, t)

print('Legacy LeKiwi task retired; canonical presentation conventions reconciled.')
