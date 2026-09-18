"""Validate the actual exported builder schemas, including raw and normalized examples."""
import json
import subprocess
from pathlib import Path
from jsonschema import Draft202012Validator
schemas = json.loads(subprocess.check_output(['node', 'tests/physics/openarm-general-contracts.mjs', '--schemas'], text=True))
for schema in schemas.values():
    Draft202012Validator.check_schema(schema)
scene_validator = Draft202012Validator(schemas['scene'])
for path in Path('tests/fixtures/general-scenes').glob('*.json'):
    scene = json.loads(path.read_text())
    assert scene_validator.is_valid(scene), (path, list(scene_validator.iter_errors(scene)))
    normalized = json.loads(subprocess.check_output(['node', '--input-type=module', '-e',
        "import fs from 'node:fs';import {normalizeGeneralScene} from './src/physics/openarm-general-scene.js';console.log(JSON.stringify(normalizeGeneralScene(JSON.parse(fs.readFileSync(process.argv[1],'utf8')))))", str(path)], text=True))
    assert scene_validator.is_valid(normalized), list(scene_validator.iter_errors(normalized))
    assert not scene_validator.is_valid({**scene, 'xml': '<mujoco/>'})
    stage = {'command':'stage', 'expected_scene_revision':'example', 'scene':scene}
    assert Draft202012Validator(schemas['manage']).is_valid(stage)
assert not Draft202012Validator(schemas['manage']).is_valid({'command':'apply','stage_id':'id','acknowledge_reset':False})
print('General laboratory scene schemas and example round trips: OK')
