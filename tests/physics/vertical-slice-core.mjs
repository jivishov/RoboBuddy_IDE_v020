import assert from 'node:assert/strict';
import { BrowserMuJoCoBackend } from '../../src/physics/browser-mujoco-backend.js';
import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord } from '../../src/physics/model-registry.js';

const methods = ['load','reset','command','step','observe','pause','resume','cancel','exportTrace','dispose'];
for (const method of methods) assert.equal(typeof BrowserMuJoCoBackend.prototype[method], 'function', `${method} must exist on BrowserMuJoCoBackend`);

const capability = capabilityRecord({
  backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
  capability: 'phase1-vertical-slice',
  evidence: MODEL_EVIDENCE.MODEL_DERIVED,
  limitations: ['Not hardware validation', 'Not a migrated production robot'],
});
assert.equal(capability.backend, 'browser-mujoco');
assert.equal(capability.evidence, 'model-derived');

console.log('Phase 1 vertical-slice contract checks: OK');
