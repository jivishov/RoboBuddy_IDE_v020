#!/usr/bin/env python3
"""Run the staged LeKiwi finalization patch without modifying persistent workflow files,
then fully retire the legacy LeKiwi task descriptor from the application catalog.
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

# Fully retire the legacy LeKiwi task descriptor; keep only LEGACY_TASK_SOURCE provenance metadata.
def read(path): return (ROOT / path).read_text()
def write(path, text): (ROOT / path).write_text(text)
def one(text, old, new, label):
    n = text.count(old)
    if n != 1: raise RuntimeError(f'{label}: expected 1, found {n}')
    return text.replace(old, new, 1)

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

print('Legacy LeKiwi task descriptor fully retired.')
