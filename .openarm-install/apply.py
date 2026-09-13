"""One-time hash-checked transport of locally tested OpenArm source changes.
All resulting files are committed as ordinary reviewable source. Remove this
transport directory and its bootstrap workflow after the source commit.
"""
import base64, hashlib, json, lzma, os
from pathlib import Path

assert os.environ['GITHUB_REPOSITORY'] == 'jivishov/RoboBuddy_IDE_v020'
assert os.environ['GITHUB_REF'] == 'refs/heads/fix/openarm-contact-workcell-webmcp'
root = Path.cwd().resolve()
bundle = root / '.openarm-install'
data = base64.b64decode(''.join((bundle / f'{i:02}.b64').read_text() for i in range(10)), validate=True)
assert hashlib.sha256(data).hexdigest() == '1272b366644c2e894ba89ffe3f242df57687b6c621753a4f6a5bc77b4978b3fd'
decoder = lzma.LZMADecompressor(memlimit=128*1024*1024)
raw = decoder.decompress(data, max_length=2_000_000)
assert decoder.eof and not decoder.unused_data
manifest = json.loads(raw)
assert len(manifest) == 37
allowed = ('models/', 'native/', 'scripts/', 'src/', 'tests/', 'docs/')
prepared = {}
for name, item in manifest.items():
    path = root / name
    assert name in ('README.md', 'package.json', 'playwright.config.mjs') or name.startswith(allowed), name
    assert not path.is_symlink() and path.resolve().is_relative_to(root) and '..' not in Path(name).parts, name
    if 'new' in item:
        assert not path.exists(), name
        result = item['new']
    else:
        before = path.read_bytes()
        assert hashlib.sha256(before).hexdigest() == item['sha256'], name
        result = before.decode('utf-8')
        end = len(result)
        for start, stop, replacement in reversed(item['edits']):
            assert 0 <= start <= stop <= end, name
            result = result[:start] + replacement + result[stop:]
            end = start
    assert hashlib.sha256(result.encode()).hexdigest() == item['resultSha256'], name
    prepared[path] = result.encode()
baseline = (root / 'models/openarm_v2/manipulation.xml').read_bytes()
assert hashlib.sha256(baseline).hexdigest() == '960ecf32c0aa7c8b2b016c6f28a7a8afe8147ce6cb1cdfd9b91f550cd4fc27dc'
source = root / 'models/openarm_v2/source/primitive-baseline.xml'
assert not source.exists()
source.parent.mkdir(parents=True, exist_ok=True)
source.write_bytes(baseline)
for path, content in prepared.items():
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
paths = list(manifest) + ['models/openarm_v2/source/primitive-baseline.xml', 'models/openarm_v2/manipulation.xml', 'models/openarm_v2/geometry.json', 'src/physics/openarm-generated.js']
Path('/tmp/openarm-paths').write_bytes(b'\0'.join(x.encode() for x in paths)+b'\0')
print('Applied verified source changes:', len(manifest))
