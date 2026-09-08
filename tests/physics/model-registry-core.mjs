import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { assertPhysicalScene } from '../../src/physics/backend-contract.js';
import { getModelPackage, listModelPackages, requireModelPackage, validateModelPackage } from '../../src/physics/model-registry.js';
import { PHASE1_MODEL_PACKAGE, SO101_PHASE2A_MODEL_PACKAGE } from '../../src/physics/model-packages.js';
import { SO101_PHASE2A_SCENE } from '../../src/physics/so101-scene.js';

assert.equal(getModelPackage('phase1-vertical-slice'), PHASE1_MODEL_PACKAGE);
assert.equal(requireModelPackage('phase1-vertical-slice').asset, 'models/vertical-slice/model.xml');
assert.equal(requireModelPackage('phase1-vertical-slice').modelId, 'phase1-vertical-slice-v1');
assert.equal(listModelPackages().filter((item) => item.id === 'phase1-vertical-slice').length, 1);

assert.equal(getModelPackage(SO101_PHASE2A_MODEL_PACKAGE.id), SO101_PHASE2A_MODEL_PACKAGE);
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.robotId, 'so101_follower');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.source.revision, '8161bba264d7fa7c99ca301e91e7fb44737676ad');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.license, 'Apache-2.0');
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.physics.timestepSeconds, 0.005);
assert.equal(SO101_PHASE2A_MODEL_PACKAGE.physics.integrator, 'implicitfast');
assert.deepEqual(SO101_PHASE2A_MODEL_PACKAGE.joints.map(({ id }) => id), ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']);
assert.deepEqual(SO101_PHASE2A_MODEL_PACKAGE.actuators.map(({ jointId }) => jointId), SO101_PHASE2A_MODEL_PACKAGE.joints.map(({ id }) => id));
assert.doesNotThrow(() => assertPhysicalScene(structuredClone(SO101_PHASE2A_SCENE)));
assert.equal(SO101_PHASE2A_SCENE.modelPackage, SO101_PHASE2A_MODEL_PACKAGE.id);
assert.equal(SO101_PHASE2A_SCENE.robotId, SO101_PHASE2A_MODEL_PACKAGE.robotId);

const so101Bytes = readFileSync(new URL('../../models/so101/model.xml', import.meta.url));
const so101Sha256 = createHash('sha256').update(so101Bytes).digest('hex');
assert.equal(so101Sha256, SO101_PHASE2A_MODEL_PACKAGE.sha256, 'SO-101 registered SHA-256 must identify the repository model bytes');
const so101Source = so101Bytes.toString('utf8');
for (const name of SO101_PHASE2A_MODEL_PACKAGE.joints.map(({ id }) => id)) {
  assert.ok(so101Source.includes(`name="${name}"`), `SO-101 MJCF must contain declared joint/actuator ${name}`);
}
assert.ok(!so101Source.includes('<freejoint'), 'Phase 2A SO-101 validation plant must not acquire a hidden free root');

assert.throws(() => requireModelPackage('https://example.invalid/model.xml'), /Unknown physical model package/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-url-package', asset: 'https://example.invalid/model.xml' }), /repository-local/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-traversal-package', asset: 'models/../secret/model.xml' }), /repository-local/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-actuator-package', actuators: [{ id: 'bad', jointId: 'missing', controllerId: 'hinge_position', command: 'position-rad' }] }), /unknown joint/);
console.log('Model registry/package contract checks: OK');
