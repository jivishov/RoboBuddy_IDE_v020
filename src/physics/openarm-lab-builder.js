export * from './openarm-lab-builder-base.js';
import { OPENARM_LAB_PROJECT_VERSION } from './openarm-lab-builder-base.js';

// Portable projects omit unavailable optional sections instead of serializing null values.
// This keeps a freshly exported blank project valid input to the bounded reopen schema and never
// implies that a task or program exists when it has not yet been authored.
export function createLabProject({ sceneSpec = null, taskSpec = null, program = null, executionProfile = null } = {}) {
  return {
    schema_version: OPENARM_LAB_PROJECT_VERSION,
    scene: sceneSpec ? structuredClone(sceneSpec) : null,
    ...(taskSpec ? { task: structuredClone(taskSpec) } : {}),
    ...(program ? { program: structuredClone(program) } : {}),
    ...(executionProfile ? { execution_profile: structuredClone(executionProfile) } : {}),
    auto_start: false,
  };
}
