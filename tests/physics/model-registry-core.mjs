import assert from 'node:assert/strict';
import { getModelPackage, listModelPackages, requireModelPackage, validateModelPackage } from '../../src/physics/model-registry.js';
import { PHASE1_MODEL_PACKAGE } from '../../src/physics/model-packages.js';

assert.equal(getModelPackage('phase1-vertical-slice'), PHASE1_MODEL_PACKAGE);
assert.equal(requireModelPackage('phase1-vertical-slice').asset, 'models/vertical-slice/model.xml');
assert.equal(requireModelPackage('phase1-vertical-slice').modelId, 'phase1-vertical-slice-v1');
assert.equal(listModelPackages().filter((item) => item.id === 'phase1-vertical-slice').length, 1);
assert.throws(() => requireModelPackage('https://example.invalid/model.xml'), /Unknown physical model package/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-url-package', asset: 'https://example.invalid/model.xml' }), /repository-local/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-traversal-package', asset: 'models/../secret/model.xml' }), /repository-local/);
assert.throws(() => validateModelPackage({ ...structuredClone(PHASE1_MODEL_PACKAGE), id: 'bad-actuator-package', actuators: [{ id: 'bad', jointId: 'missing', controllerId: 'hinge_position', command: 'position-rad' }] }), /unknown joint/);
console.log('Model registry/package contract checks: OK');
