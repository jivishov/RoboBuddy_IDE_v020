// Standalone image-assisted OpenArm Lab Builder contracts.
// This module does not depend on the fleet physics-migration plan or a hidden reference task.

export const OPENARM_LAB_BUILDER_SCENE_ID = 'openarm-lab-builder-empty-v1';
export const OPENARM_LAB_BUILDER_SCENE_REVISION = 'openarm-lab-builder-empty-v1';
export const OPENARM_LAB_SCENE_VERSION = 'robobuddy.lab.scene.v1';
export const OPENARM_LAB_TASK_VERSION = 'robobuddy.lab.task.v1';
export const OPENARM_LAB_PROJECT_VERSION = 'robobuddy.lab.project.v1';
export const OPENARM_LAB_EXECUTION_VERSION = 'robobuddy.lab.execution.v1';

export const LAB_QUANTITY_SOURCES = Object.freeze(['source_provided', 'user_measured', 'image_estimated', 'fitted', 'assumed']);
export const LAB_REFERENCE_MODES = Object.freeze(['none', 'external_reference', 'local_file', 'synthetic_fixture']);
export const LAB_SUPPORTED_TASK_TYPES = Object.freeze(['dry_transfer']);

const REFERENCE_GEOM_IDS = new Set([
  'cell_table', 'left_source_support', 'left_hotplate', 'right_source_support',
  'right_ring_post', 'right_ring_gauze', 'right_ring_bracket',
]);
const REFERENCE_BODY_IDS = new Set(['flask', 'beaker']);
const plain = (value, label) => {
  if (!value || typeof value !== 'object' || Array.isArray(value) || ![Object.prototype, null].includes(Object.getPrototypeOf(value))) throw new TypeError(`${label} must be a plain object`);
};
const finite = (value, label) => {
  if (typeof value !== 'number' || !Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
  return value;
};
const vec3 = (value, label) => {
  if (!Array.isArray(value) || value.length !== 3) throw new TypeError(`${label} must contain three numbers`);
  return value.map((v, i) => finite(v, `${label}[${i}]`));
};
const text = (value, label, max = 240) => {
  if (typeof value !== 'string' || !value.trim() || value.length > max || /[\x00-\x08\x0b\x0c\x0e-\x1f]/.test(value)) throw new TypeError(`${label} must be printable text up to ${max} characters`);
  return value.trim();
};
const onlyKeys = (value, allowed, label) => {
  for (const key of Object.keys(value)) if (!allowed.includes(key)) throw new TypeError(`Unknown ${label} field ${key}`);
};
const id = (value, label) => {
  if (typeof value !== 'string' || !/^[a-z][a-z0-9_]{0,31}$/.test(value)) throw new TypeError(`${label} must be a lowercase identifier`);
  return value;
};

export function isOpenArmReferenceTaskGeometry(record) {
  const geomId = typeof record === 'string' ? record : record?.id;
  const bodyId = typeof record === 'object' ? record?.bodyId : null;
  return REFERENCE_GEOM_IDS.has(geomId) || (typeof geomId === 'string' && geomId.startsWith('table_leg_')) || REFERENCE_BODY_IDS.has(bodyId);
}

function removeNamedBody(xml, bodyName) {
  const token = `<body name="${bodyName}"`;
  const start = xml.indexOf(token);
  if (start < 0) throw new Error(`Lab Builder base expected reference body ${bodyName}`);
  let cursor = start;
  let depth = 0;
  while (cursor < xml.length) {
    const open = xml.indexOf('<body', cursor);
    const close = xml.indexOf('</body>', cursor);
    if (close < 0) throw new Error(`Unbalanced MuJoCo body while removing ${bodyName}`);
    if (open >= 0 && open < close) {
      const end = xml.indexOf('>', open);
      if (end < 0) throw new Error(`Malformed MuJoCo body while removing ${bodyName}`);
      const selfClosing = xml[end - 1] === '/';
      if (!selfClosing) depth += 1;
      cursor = end + 1;
      continue;
    }
    depth -= 1;
    cursor = close + '</body>'.length;
    if (depth === 0) return xml.slice(0, start) + xml.slice(cursor);
    if (depth < 0) break;
  }
  throw new Error(`Could not isolate reference body ${bodyName}`);
}

export function stripOpenArmReferenceWorkcellXml(sourceXml) {
  if (typeof sourceXml !== 'string' || !sourceXml.includes('<mujoco') || !sourceXml.includes('name="openarm_mount"') || !sourceXml.includes('name="floor"')) throw new TypeError('Lab Builder requires the verified OpenArm source-derived MuJoCo model');
  let xml = sourceXml.replace(/<geom\b[^>]*\bname="([^"]+)"[^>]*\/>/g, (tag, name) => {
    if (REFERENCE_GEOM_IDS.has(name) || name.startsWith('table_leg_')) return '';
    return tag;
  });
  for (const bodyName of REFERENCE_BODY_IDS) xml = removeNamedBody(xml, bodyName);
  for (const forbidden of [...REFERENCE_GEOM_IDS, 'flask', 'beaker']) {
    if (xml.includes(`name="${forbidden}"`)) throw new Error(`Reference-task dependency ${forbidden} remained in Lab Builder base`);
  }
  if (/name="table_leg_[^"]+"/.test(xml)) throw new Error('Reference-task table legs remained in Lab Builder base');
  if (!xml.includes('name="floor"') || !xml.includes('name="openarm_mount"')) throw new Error('Lab Builder stripping removed a required boundary condition');
  return xml;
}

export function builderPresentationGeometry(geometry) {
  plain(geometry, 'geometry');
  return {
    ...structuredClone(geometry),
    geoms: (geometry.geoms || []).filter(record => !isOpenArmReferenceTaskGeometry(record)),
  };
}

export function validateReferenceProvenance(raw = { mode: 'none' }) {
  plain(raw, 'reference');
  onlyKeys(raw, ['mode', 'label', 'content_sha256', 'dimensions', 'notes', 'image_pixels_available'], 'reference');
  if (!LAB_REFERENCE_MODES.includes(raw.mode)) throw new TypeError('reference.mode is unsupported');
  if (raw.label !== undefined) text(raw.label, 'reference.label', 120);
  if (raw.notes !== undefined) text(raw.notes, 'reference.notes', 1000);
  if (raw.content_sha256 !== undefined && !/^[a-f0-9]{64}$/.test(raw.content_sha256)) throw new TypeError('reference.content_sha256 must be a SHA-256 hex digest computed from available bytes');
  if (raw.image_pixels_available !== undefined && typeof raw.image_pixels_available !== 'boolean') throw new TypeError('reference.image_pixels_available must be boolean');
  let dimensions;
  if (raw.dimensions !== undefined) {
    plain(raw.dimensions, 'reference.dimensions');
    onlyKeys(raw.dimensions, ['known_length_m', 'description', 'source'], 'reference.dimensions');
    const known = finite(raw.dimensions.known_length_m, 'reference.dimensions.known_length_m');
    if (known <= 0 || known > 10) throw new RangeError('known reference length must be 0..10 m');
    const source = raw.dimensions.source;
    if (!LAB_QUANTITY_SOURCES.includes(source)) throw new TypeError('reference dimension source is unsupported');
    dimensions = { known_length_m: known, description: text(raw.dimensions.description, 'reference.dimensions.description', 160), source };
  }
  if (raw.content_sha256 && raw.image_pixels_available !== true) throw new TypeError('A content hash may be recorded only when image bytes were actually available');
  return { mode: raw.mode, ...(raw.label ? { label: raw.label.trim() } : {}), ...(raw.content_sha256 ? { content_sha256: raw.content_sha256 } : {}), ...(dimensions ? { dimensions } : {}), ...(raw.notes ? { notes: raw.notes.trim() } : {}), image_pixels_available: raw.image_pixels_available === true };
}

export function validateQuantityEvidence(raw, label) {
  plain(raw, label);
  onlyKeys(raw, ['source', 'source_detail', 'uncertainty_m', 'assumption'], label);
  if (!LAB_QUANTITY_SOURCES.includes(raw.source)) throw new TypeError(`${label}.source is unsupported`);
  const result = { source: raw.source };
  if (raw.source_detail !== undefined) result.source_detail = text(raw.source_detail, `${label}.source_detail`, 200);
  if (raw.assumption !== undefined) result.assumption = text(raw.assumption, `${label}.assumption`, 500);
  if (raw.uncertainty_m !== undefined) {
    const uncertainty = finite(raw.uncertainty_m, `${label}.uncertainty_m`);
    if (uncertainty < 0 || uncertainty > 1) throw new RangeError(`${label}.uncertainty_m must be 0..1 m`);
    result.uncertainty_m = uncertainty;
  }
  return result;
}

export function validateLabSceneSpec(raw, validateEquipment) {
  plain(raw, 'SceneSpec');
  onlyKeys(raw, ['schema_version', 'id', 'reference', 'assets', 'assumptions', 'adaptation_of', 'adaptation_notes'], 'SceneSpec');
  if (raw.schema_version !== OPENARM_LAB_SCENE_VERSION) throw new TypeError(`SceneSpec.schema_version must be ${OPENARM_LAB_SCENE_VERSION}`);
  const sceneId = id(raw.id, 'SceneSpec.id');
  const reference = validateReferenceProvenance(raw.reference || { mode: 'none' });
  if (!Array.isArray(raw.assets) || raw.assets.length > 24) throw new RangeError('SceneSpec.assets must contain 0..24 assets');
  if (typeof validateEquipment !== 'function') throw new TypeError('SceneSpec requires the equipment validator');
  const normalizedEquipment = validateEquipment(raw.assets.map((asset, index) => {
    plain(asset, `SceneSpec.assets[${index}]`);
    const equipmentKeys = ['id', 'label', 'kind', 'position_m', 'yaw_rad', 'dimensions_m', 'mass_kg', 'dynamic', 'parts'];
    onlyKeys(asset, [...equipmentKeys, 'supported_by', 'quantity_evidence', 'role'], `SceneSpec.assets[${index}]`);
    return Object.fromEntries(Object.entries(asset).filter(([key]) => equipmentKeys.includes(key)));
  }), { builderMode: true });
  const normalizedAssets = normalizedEquipment.map((equipment, index) => {
    const original = raw.assets[index];
    const supportedBy = original.supported_by === undefined ? null : id(original.supported_by, `SceneSpec.assets[${index}].supported_by`);
    const evidence = original.quantity_evidence === undefined ? { source: 'assumed', assumption: 'No quantity provenance supplied by the authoring agent.' } : validateQuantityEvidence(original.quantity_evidence, `SceneSpec.assets[${index}].quantity_evidence`);
    const role = original.role === undefined ? null : text(original.role, `SceneSpec.assets[${index}].role`, 120);
    return { ...equipment, ...(supportedBy ? { supported_by: supportedBy } : {}), quantity_evidence: evidence, ...(role ? { role } : {}) };
  });
  const ids = new Set(normalizedAssets.map(asset => asset.id));
  for (const asset of normalizedAssets) if (asset.supported_by && !ids.has(asset.supported_by)) throw new TypeError(`Asset ${asset.id} references unknown supported_by ${asset.supported_by}`);
  const assumptions = raw.assumptions === undefined ? [] : raw.assumptions;
  if (!Array.isArray(assumptions) || assumptions.length > 32) throw new RangeError('SceneSpec.assumptions must contain 0..32 entries');
  const normalizedAssumptions = assumptions.map((entry, index) => text(entry, `SceneSpec.assumptions[${index}]`, 500));
  if (raw.adaptation_of !== undefined) id(raw.adaptation_of, 'SceneSpec.adaptation_of');
  const adaptationNotes = raw.adaptation_notes === undefined ? undefined : text(raw.adaptation_notes, 'SceneSpec.adaptation_notes', 1000);
  return { schema_version: OPENARM_LAB_SCENE_VERSION, id: sceneId, reference, assets: normalizedAssets, assumptions: normalizedAssumptions, ...(raw.adaptation_of ? { adaptation_of: raw.adaptation_of } : {}), ...(adaptationNotes ? { adaptation_notes: adaptationNotes } : {}) };
}

export function validateLabTaskSpec(raw, sceneSpec) {
  plain(raw, 'TaskSpec');
  onlyKeys(raw, ['schema_version', 'id', 'type', 'object_id', 'receiver_id', 'side', 'required_action_sequence', 'tolerances', 'prohibited_contacts'], 'TaskSpec');
  if (raw.schema_version !== OPENARM_LAB_TASK_VERSION) throw new TypeError(`TaskSpec.schema_version must be ${OPENARM_LAB_TASK_VERSION}`);
  const taskId = id(raw.id, 'TaskSpec.id');
  if (!LAB_SUPPORTED_TASK_TYPES.includes(raw.type)) return { supported: false, capability: 'unsupported', reason: `Task type ${String(raw.type)} is not implemented; supported task family: dry_transfer.` };
  if (!sceneSpec) throw new TypeError('A validated SceneSpec is required before authoring a task');
  const objectId = id(raw.object_id, 'TaskSpec.object_id');
  const receiverId = id(raw.receiver_id, 'TaskSpec.receiver_id');
  const object = sceneSpec.assets.find(asset => asset.id === objectId);
  const receiver = sceneSpec.assets.find(asset => asset.id === receiverId);
  if (!object || !['vial', 'block'].includes(object.kind) || object.dynamic !== true) throw new TypeError('TaskSpec.object_id must identify a dynamic vial or block');
  if (!receiver || !['receiver', 'tray', 'rack'].includes(receiver.kind) || receiver.dynamic === true) throw new TypeError('TaskSpec.receiver_id must identify a fixed accessible receiver, tray, or rack');
  if (!['left', 'right'].includes(raw.side)) throw new TypeError('TaskSpec.side must be left or right');
  const t = raw.tolerances || {};
  plain(t, 'TaskSpec.tolerances');
  onlyKeys(t, ['position_m', 'orientation_rad', 'settle_speed_ms', 'settle_angular_speed_rads', 'settle_dwell_s', 'retreat_m', 'max_penetration_m'], 'TaskSpec.tolerances');
  const bounded = (value, fallback, min, max, label) => {
    const x = value === undefined ? fallback : finite(value, label);
    if (x < min || x > max) throw new RangeError(`${label} must be ${min}..${max}`);
    return x;
  };
  const tolerances = {
    position_m: bounded(t.position_m, .012, .002, .08, 'position_m'),
    orientation_rad: bounded(t.orientation_rad, .35, .03, Math.PI, 'orientation_rad'),
    settle_speed_ms: bounded(t.settle_speed_ms, .02, .001, .2, 'settle_speed_ms'),
    settle_angular_speed_rads: bounded(t.settle_angular_speed_rads, .25, .01, 3, 'settle_angular_speed_rads'),
    settle_dwell_s: bounded(t.settle_dwell_s, .20, .04, 2, 'settle_dwell_s'),
    retreat_m: bounded(t.retreat_m, .06, .02, .25, 'retreat_m'),
    max_penetration_m: bounded(t.max_penetration_m, .002, .0002, .01, 'max_penetration_m'),
  };
  const prohibited = raw.prohibited_contacts === undefined ? [] : raw.prohibited_contacts;
  if (!Array.isArray(prohibited) || prohibited.length > 32 || prohibited.some(value => typeof value !== 'string' || value.length > 100)) throw new TypeError('TaskSpec.prohibited_contacts must be a bounded string list');
  return { supported: true, task: { schema_version: OPENARM_LAB_TASK_VERSION, id: taskId, type: 'dry_transfer', object_id: objectId, receiver_id: receiverId, side: raw.side, required_action_sequence: raw.required_action_sequence !== false, tolerances, prohibited_contacts: prohibited.map(value => value.trim()) } };
}

export function equipmentFromSceneSpec(sceneSpec) {
  return sceneSpec.assets.map(asset => {
    const { supported_by, quantity_evidence, role, ...equipment } = asset;
    return equipment;
  });
}

export function createLabProject({ sceneSpec = null, taskSpec = null, program = null, executionProfile = null } = {}) {
  return {
    schema_version: OPENARM_LAB_PROJECT_VERSION,
    scene: sceneSpec ? structuredClone(sceneSpec) : null,
    task: taskSpec ? structuredClone(taskSpec) : null,
    program: program ? structuredClone(program) : null,
    execution_profile: executionProfile ? structuredClone(executionProfile) : null,
    auto_start: false,
  };
}

export function openArmLabBuilderScene(modelPackage) {
  return Object.freeze({
    schemaVersion: 1,
    id: OPENARM_LAB_BUILDER_SCENE_ID,
    revision: OPENARM_LAB_BUILDER_SCENE_REVISION,
    modelPackage,
    robotId: 'openarm_v2_bimanual',
    controllers: Object.freeze(['openarm_v2_position']),
    taskGoal: null,
    workspaceMode: 'lab_builder',
    boundaryConditions: Object.freeze({ floor: 'world-fixed plane', openarmMount: 'world-fixed source-derived mount/pedestal; placement is setup, never trial rescue' }),
  });
}
