import { PHYSICS_BACKEND_API_VERSION } from './backend-contract.js';
import { SO101_PHASE2A_MODEL_PACKAGE } from './model-packages.js';

export const SO101_PHASE2A_SCENE = Object.freeze({
  schemaVersion: PHYSICS_BACKEND_API_VERSION,
  id: 'phase2a-so101-articulation',
  revision: 'phase2a-so101-articulation-v1',
  robotId: SO101_PHASE2A_MODEL_PACKAGE.robotId,
  modelPackage: SO101_PHASE2A_MODEL_PACKAGE.id,
  legacyTaskId: null,
  physics: Object.freeze({
    timestepSeconds: SO101_PHASE2A_MODEL_PACKAGE.physics.timestepSeconds,
    integrator: SO101_PHASE2A_MODEL_PACKAGE.physics.integrator,
    iterations: SO101_PHASE2A_MODEL_PACKAGE.physics.iterations,
    lsIterations: SO101_PHASE2A_MODEL_PACKAGE.physics.lsIterations,
  }),
  fixtures: Object.freeze([]),
  objects: Object.freeze([]),
  controllers: Object.freeze([...SO101_PHASE2A_MODEL_PACKAGE.controllers]),
  taskGoal: null,
});
