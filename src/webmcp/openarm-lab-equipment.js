import { normalizeOpenArmLabEquipment, OPENARM_LAB_EQUIPMENT_CATALOG, OPENARM_LAB_EQUIPMENT_LIMITS, OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION } from '../physics/openarm-lab-equipment.js';
import { WebMcpDomainError } from './agent-facade.js';

function invalid(message) { throw new WebMcpDomainError('INVALID_ARGUMENT', message, { retryable: false }); }
function plain(value, label = 'input') { if (!value || typeof value !== 'object' || Array.isArray(value)) invalid(`${label} must be an object.`); }
function onlyKeys(value, allowed, label = 'input') { for (const key of Object.keys(value)) if (!allowed.includes(key)) invalid(`Unexpected ${label} field: ${key}.`); }
function sameAuthority(a, b) { return Boolean(a && b && a.sessionId === b.sessionId && Number(a.epoch) === Number(b.epoch) && a.sceneRevision === b.sceneRevision && a.robotId === b.robotId); }

const boxFixed = {
  type: 'object',
  properties: {
    id: { type: 'string', pattern: '^[a-z][a-z0-9_-]{0,31}$' },
    shape: { type: 'string', const: 'box' },
    mobility: { type: 'string', const: 'fixed' },
    positionM: { type: 'array', minItems: 3, maxItems: 3, items: { type: 'number' } },
    sizeM: { type: 'array', minItems: 3, maxItems: 3, items: { type: 'number', minimum: OPENARM_LAB_EQUIPMENT_LIMITS.boxSizeM[0], maximum: OPENARM_LAB_EQUIPMENT_LIMITS.boxSizeM[1] } },
  },
  required: ['id', 'shape', 'mobility', 'positionM', 'sizeM'], additionalProperties: false,
};
const boxFree = {
  type: 'object',
  properties: { ...boxFixed.properties, mobility: { type: 'string', const: 'free' }, massKg: { type: 'number', minimum: OPENARM_LAB_EQUIPMENT_LIMITS.freeMassKg[0], maximum: OPENARM_LAB_EQUIPMENT_LIMITS.freeMassKg[1] } },
  required: ['id', 'shape', 'mobility', 'positionM', 'sizeM', 'massKg'], additionalProperties: false,
};
const cylinderFixed = {
  type: 'object',
  properties: {
    id: { type: 'string', pattern: '^[a-z][a-z0-9_-]{0,31}$' },
    shape: { type: 'string', const: 'cylinder' },
    mobility: { type: 'string', const: 'fixed' },
    positionM: { type: 'array', minItems: 3, maxItems: 3, items: { type: 'number' } },
    sizeM: {
      type: 'array', minItems: 2, maxItems: 2,
      prefixItems: [
        { type: 'number', minimum: OPENARM_LAB_EQUIPMENT_LIMITS.cylinderRadiusM[0], maximum: OPENARM_LAB_EQUIPMENT_LIMITS.cylinderRadiusM[1] },
        { type: 'number', minimum: OPENARM_LAB_EQUIPMENT_LIMITS.cylinderHeightM[0], maximum: OPENARM_LAB_EQUIPMENT_LIMITS.cylinderHeightM[1] },
      ], items: false,
    },
  },
  required: ['id', 'shape', 'mobility', 'positionM', 'sizeM'], additionalProperties: false,
};
const cylinderFree = {
  type: 'object',
  properties: { ...cylinderFixed.properties, mobility: { type: 'string', const: 'free' }, massKg: { type: 'number', minimum: OPENARM_LAB_EQUIPMENT_LIMITS.freeMassKg[0], maximum: OPENARM_LAB_EQUIPMENT_LIMITS.freeMassKg[1] } },
  required: ['id', 'shape', 'mobility', 'positionM', 'sizeM', 'massKg'], additionalProperties: false,
};
const itemSchema = { oneOf: [boxFixed, boxFree, cylinderFixed, cylinderFree] };

export function createOpenArmLabEquipmentSchema() {
  return {
    type: 'object',
    oneOf: [
      { type: 'object', properties: { schema_version: { type: 'string', const: OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION }, command: { type: 'string', const: 'inspect_catalog' } }, required: ['schema_version', 'command'], additionalProperties: false },
      { type: 'object', properties: { schema_version: { type: 'string', const: OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION }, command: { type: 'string', const: 'inspect_scene' } }, required: ['schema_version', 'command'], additionalProperties: false },
      { type: 'object', properties: { schema_version: { type: 'string', const: OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION }, command: { type: 'string', const: 'replace' }, items: { type: 'array', maxItems: OPENARM_LAB_EQUIPMENT_LIMITS.maxItems, items: itemSchema } }, required: ['schema_version', 'command', 'items'], additionalProperties: false },
      { type: 'object', properties: { schema_version: { type: 'string', const: OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION }, command: { type: 'string', const: 'clear' } }, required: ['schema_version', 'command'], additionalProperties: false },
    ],
    additionalProperties: false,
  };
}

export function getOpenArmLabEquipmentDefinition(facade) {
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || !context.simulationReady || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco') return null;
  return {
    name: 'configure_openarm_lab_equipment',
    title: 'Build bounded lab equipment in the OpenArm physical scene',
    description: `Inspect or replace a temporary ${OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION} equipment set compiled into the same MuJoCo world as OpenArm. The surface accepts only bounded box/cylinder rigid bodies: fixed fixtures or free dynamic objects. It cannot accept arbitrary XML, meshes, URLs, scripts, plugins, welds, source edits, saves, publishing, or hardware transport. Replacing equipment reloads the physical scene and resets simulation state.`,
    inputSchema: createOpenArmLabEquipmentSchema(),
  };
}

function capture(facade, expectedEpoch) {
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco' || !context.simulationReady) throw new WebMcpDomainError('PROFILE_MISMATCH', 'This tool requires the active ready OpenArm V2 physical workspace.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'Finish or stop the active Python run before changing OpenArm lab equipment.', { retryable: true });
  const authority = facade.app.sim.getPhysicalAuthorityToken?.();
  if (!authority?.sessionId) throw new WebMcpDomainError('SIMULATION_NOT_READY', 'OpenArm PhysicsSession authority is unavailable.', { retryable: true });
  return Object.freeze({ workspaceGeneration: context.workspaceGeneration, simulatorEpoch: context.simulatorEpoch, authority: Object.freeze({ ...authority }) });
}
function assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange = false } = {}) {
  if (signal?.aborted) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The OpenArm lab-equipment call was cancelled.', { retryable: true });
  facade.assertActive(expectedEpoch);
  const context = facade.getRegistrationContext();
  if (context.workspaceStatus !== 'ready' || context.profileId !== 'openarm' || context.simulationMode !== 'physical_mujoco' || context.workspaceGeneration !== baseline.workspaceGeneration || context.simulatorEpoch !== baseline.simulatorEpoch) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The active OpenArm workspace or simulator changed during lab-equipment configuration.', { retryable: true });
  if (!allowAuthorityChange && !sameAuthority(baseline.authority, facade.app.sim.getPhysicalAuthorityToken?.())) throw new WebMcpDomainError('OPERATION_CANCELLED', 'The OpenArm PhysicsSession epoch changed during lab-equipment configuration.', { retryable: true });
  if (facade.app.getExecutionState() !== 'idle') throw new WebMcpDomainError('SIMULATION_BUSY', 'A Python run acquired the simulation during lab-equipment configuration.', { retryable: true });
}
function parse(input) {
  plain(input);
  if (input.schema_version !== OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION) invalid(`schema_version must be ${OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION}.`);
  if (!['inspect_catalog', 'inspect_scene', 'replace', 'clear'].includes(input.command)) invalid('command must be inspect_catalog, inspect_scene, replace, or clear.');
  if (input.command === 'replace') {
    onlyKeys(input, ['schema_version', 'command', 'items']);
    try { return { command: 'replace', items: normalizeOpenArmLabEquipment(input.items) }; }
    catch (error) { invalid(String(error?.message || error)); }
  }
  onlyKeys(input, ['schema_version', 'command']);
  return { command: input.command };
}

export async function executeOpenArmLabEquipment(facade, input, signal, expectedEpoch) {
  const parsed = parse(input);
  const baseline = capture(facade, expectedEpoch);
  assertCurrent(facade, baseline, expectedEpoch, signal);
  if (parsed.command === 'inspect_catalog') return { ok: true, profileId: 'openarm', command: parsed.command, catalog: structuredClone(OPENARM_LAB_EQUIPMENT_CATALOG), physicalAuthority: baseline.authority };
  if (parsed.command === 'inspect_scene') return { ok: true, profileId: 'openarm', command: parsed.command, scene: facade.app.sim.getLabEquipmentState(), physicalAuthority: baseline.authority };
  if (facade.activeControlId) throw new WebMcpDomainError('COMMAND_CONFLICT', 'Another bounded robot-control call is active.', { retryable: true });
  const controlId = `webmcp-openarm-lab-${expectedEpoch}-${++facade.controlSequence}`;
  facade.activeControlId = controlId;
  try {
    const items = parsed.command === 'clear' ? [] : parsed.items;
    const scene = await facade.app.sim.configureLabEquipment(items);
    assertCurrent(facade, baseline, expectedEpoch, signal, { allowAuthorityChange: true });
    facade.app.setStatus?.(`Agent ${items.length ? 'rebuilt' : 'cleared'} temporary OpenArm lab equipment`);
    facade.app.renderPanels?.();
    return { ok: true, profileId: 'openarm', command: parsed.command, scene, physicalAuthority: facade.app.sim.getPhysicalAuthorityToken?.(), reset: true, hardwareValidated: false };
  } catch (error) {
    if (error instanceof WebMcpDomainError) throw error;
    throw new WebMcpDomainError('SIMULATION_REJECTED', String(error?.message || error).slice(0, 360), { retryable: true, details: { profileId: 'openarm' } });
  } finally {
    if (facade.activeControlId === controlId) facade.activeControlId = null;
  }
}
