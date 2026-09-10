#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
worker_path = ROOT / 'src/physics/microduck-mujoco-worker.js'
text = worker_path.read_text()

marker = "async function load(modelPackage) {\n"
if text.count(marker) != 1:
    raise SystemExit(f'load marker drifted: {text.count(marker)}')

helper = r'''function meshDependenciesFromXml(xml) {
  const compiler = xml.match(/<compiler\b[^>]*\bmeshdir="([^"]+)"/i);
  const meshDir = compiler?.[1] || '';
  if (meshDir !== 'assets') throw new Error(`MicroDuck source model must declare compiler meshdir="assets", got ${meshDir || '<none>'}`);
  const files = [...xml.matchAll(/<mesh\b[^>]*\bfile="([^"]+)"/gi)].map((match) => match[1]);
  if (!files.length) throw new Error('MicroDuck source model declares no external mesh files');
  const unique = [...new Set(files)];
  for (const file of unique) {
    if (!/^[A-Za-z0-9._-]+\.stl$/i.test(file) || file.includes('..') || file.includes('/') || file.includes('\\')) {
      throw new Error(`Worker rejected unsafe MicroDuck mesh dependency: ${file}`);
    }
  }
  return unique;
}

async function buildModelVfs(mj, xml, modelUrl) {
  if (typeof mj?.MjVFS !== 'function') throw new Error('Bundled MuJoCo runtime does not expose MjVFS');
  const files = meshDependenciesFromXml(xml);
  const modelDirectory = new URL('./', modelUrl);
  const assetDirectory = new URL('assets/', modelDirectory);
  if (assetDirectory.origin !== self.location.origin) throw new Error('MicroDuck mesh directory must be same-origin');
  const vfs = new mj.MjVFS();
  try {
    await Promise.all(files.map(async (file) => {
      const url = new URL(file, assetDirectory);
      if (url.origin !== self.location.origin || !url.pathname.startsWith(assetDirectory.pathname)) {
        throw new Error(`Worker rejected cross-origin or escaping MicroDuck mesh dependency: ${file}`);
      }
      const response = await fetch(url.href, { cache: 'no-store' });
      if (!response.ok) throw new Error(`MicroDuck mesh ${file} returned HTTP ${response.status}`);
      const bytes = new Uint8Array(await response.arrayBuffer());
      if (!bytes.byteLength) throw new Error(`MicroDuck mesh ${file} is empty`);
      // The source MJCF declares meshdir="assets". This is the exact VFS key
      // resolved by MuJoCo; no path rewriting or primitive substitution occurs.
      vfs.addBuffer(`assets/${file}`, bytes);
    }));
    return vfs;
  } catch (error) {
    try { vfs.delete?.(); } catch {}
    throw error;
  }
}

'''
text = text.replace(marker, helper + marker, 1)

old = """  const modelSha256 = await sha256Text(xml); if (modelSha256 !== modelPackage.sha256) throw new Error(`Model SHA-256 mismatch for ${modelPackage.id}`);\n  model = mj.from_xml_string(xml); if (!model) throw new Error(`MuJoCo failed to compile ${modelPackage.id}`);\n  data = new mj.MjData(model); if (!data) throw new Error(`MuJoCo failed to allocate data for ${modelPackage.id}`);\n"""
new = """  const modelSha256 = await sha256Text(xml); if (modelSha256 !== modelPackage.sha256) throw new Error(`Model SHA-256 mismatch for ${modelPackage.id}`);\n  const vfs = await buildModelVfs(mj, xml, modelUrl);\n  try { model = mj.MjModel.from_xml_string(xml, vfs); } finally { try { vfs.delete?.(); } catch {} }\n  if (!model) throw new Error(`MuJoCo failed to compile ${modelPackage.id}`);\n  data = new mj.MjData(model); if (!data) throw new Error(`MuJoCo failed to allocate data for ${modelPackage.id}`);\n"""
if text.count(old) != 1:
    raise SystemExit(f'XML-only compile block drifted: {text.count(old)}')
text = text.replace(old, new, 1)
worker_path.write_text(text)

test_path = ROOT / 'tests/physics/microduck-browser-mesh-vfs-core.mjs'
test_path.write_text(r'''import { readFileSync } from 'node:fs';

const worker = readFileSync(new URL('../../src/physics/microduck-mujoco-worker.js', import.meta.url), 'utf8');
const walk = readFileSync(new URL('../../models/microduck/walk.xml', import.meta.url), 'utf8');

function assert(condition, message) { if (!condition) throw new Error(message); }

assert(/<compiler[^>]*meshdir="assets"/.test(walk), 'MicroDuck exact-mesh MJCF no longer declares meshdir=assets');
assert((walk.match(/<mesh\b[^>]*\bfile="[^"]+\.stl"/g) || []).length >= 10, 'MicroDuck walk plant no longer carries external STL assets');
assert(/new mj\.MjVFS\(\)/.test(worker), 'MicroDuck worker no longer constructs a MuJoCo VFS');
assert(/vfs\.addBuffer\(`assets\/\$\{file\}`/.test(worker), 'MicroDuck worker no longer mounts meshdir assets into the VFS');
assert(/mj\.MjModel\.from_xml_string\(xml, vfs\)/.test(worker), 'MicroDuck worker compiled XML without the mesh VFS');
assert(/url\.origin !== self\.location\.origin/.test(worker), 'MicroDuck worker lost same-origin dependency enforcement');
assert(/Worker rejected unsafe MicroDuck mesh dependency/.test(worker), 'MicroDuck worker lost dependency path validation');

console.log('MicroDuck browser exact-mesh VFS loader contract: OK');
''')

workflow_path = ROOT / '.github/workflows/microduck-phase5c-validate.yml'
w = workflow_path.read_text()
if 'microduck-browser-mesh-vfs-core.mjs' not in w:
    syntax_marker = "          node --check tests/physics/microduck-lifecycle-core.mjs\n"
    if w.count(syntax_marker) != 1:
        raise SystemExit('Phase5C syntax insertion marker drifted')
    w = w.replace(syntax_marker, syntax_marker + "          node --check tests/physics/microduck-browser-mesh-vfs-core.mjs\n", 1)
    gate_marker = "      - name: MicroDuck session, setup path, cadence, cancellation and lifecycle gates\n        run: node tests/physics/microduck-lifecycle-core.mjs\n"
    if w.count(gate_marker) != 1:
        raise SystemExit('Phase5C VFS gate insertion marker drifted')
    w = w.replace(gate_marker, gate_marker + "\n      - name: MicroDuck exact-mesh browser VFS loading contract\n        run: node tests/physics/microduck-browser-mesh-vfs-core.mjs\n", 1)
workflow_path.write_text(w)

print('MicroDuck browser exact-mesh VFS patch applied')
