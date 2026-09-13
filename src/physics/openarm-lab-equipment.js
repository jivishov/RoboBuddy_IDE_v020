const ID_RE = /^[a-z][a-z0-9_-]{0,31}$/;
const SHAPES = new Set(['box', 'cylinder']);
const MOBILITIES = new Set(['fixed', 'free']);
const MAX_ITEMS = 12;
const MAX_FREE_ITEMS = 6;
const TABLETOP = Object.freeze({
  xM: Object.freeze([0.0, 0.82]),
  yM: Object.freeze([-0.55, 0.55]),
  zM: 1.005,
});
const WORKSPACE = Object.freeze({
  xM: Object.freeze([0.20, 0.78]),
  yM: Object.freeze([-0.48, 0.48]),
  zM: Object.freeze([TABLETOP.zM, 1.48]),
});

export const OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION = 'robobuddy.openarm.lab.v1';
export const OPENARM_LAB_EQUIPMENT_LIMITS = Object.freeze({
  maxItems: MAX_ITEMS,
  maxFreeItems: MAX_FREE_ITEMS,
  tabletop: TABLETOP,
  workspace: WORKSPACE,
  boxSizeM: Object.freeze([0.005, 0.35]),
  cylinderRadiusM: Object.freeze([0.003, 0.12]),
  cylinderHeightM: Object.freeze([0.005, 0.35]),
  freeMassKg: Object.freeze([0.005, 2.0]),
});

export const OPENARM_LAB_EQUIPMENT_CATALOG = Object.freeze({
  schemaVersion: OPENARM_LAB_EQUIPMENT_SCHEMA_VERSION,
  semantics: 'Validated rigid-body primitives compiled into the same OpenArm MuJoCo world. fixed items are fixtures; free items are dynamic rigid bodies. Item extents must remain above and within the physical tabletop. No arbitrary XML, mesh URL, script, plugin, weld, snap, teleport, liquid, thermal or hardware-control surface is exposed.',
  shapes: Object.freeze({
    box: Object.freeze({ sizeM: '[x, y, z] full dimensions in metres' }),
    cylinder: Object.freeze({ sizeM: '[radius, height] in metres; cylinder axis is MuJoCo +Z' }),
  }),
  mobilities: Object.freeze(['fixed', 'free']),
  examples: Object.freeze([
    Object.freeze({ id: 'extra_hotplate', shape: 'box', mobility: 'fixed', positionM: [0.62, 0.30, 1.025], sizeM: [0.12, 0.10, 0.04] }),
    Object.freeze({ id: 'sample_vial', shape: 'cylinder', mobility: 'free', positionM: [0.50, 0.30, 1.055], sizeM: [0.012, 0.08], massKg: 0.025 }),
  ]),
  limits: OPENARM_LAB_EQUIPMENT_LIMITS,
});

function finiteNumber(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new TypeError(`${label} must be finite`);
  return number;
}
function bounded(value, [minimum, maximum], label) {
  const number = finiteNumber(value, label);
  if (number < minimum || number > maximum) throw new RangeError(`${label} must be within ${minimum}..${maximum}`);
  return number;
}
function vector(value, length, label) {
  if (!Array.isArray(value) || value.length !== length) throw new TypeError(`${label} must contain ${length} numbers`);
  return value.map((entry, index) => finiteNumber(entry, `${label}[${index}]`));
}
function normalizePosition(value, label) {
  const result = vector(value, 3, label);
  result[0] = bounded(result[0], WORKSPACE.xM, `${label}[0]`);
  result[1] = bounded(result[1], WORKSPACE.yM, `${label}[1]`);
  result[2] = bounded(result[2], WORKSPACE.zM, `${label}[2]`);
  return result;
}
function normalizeSize(shape, value, label) {
  if (shape === 'box') return vector(value, 3, label).map((entry, index) => bounded(entry, OPENARM_LAB_EQUIPMENT_LIMITS.boxSizeM, `${label}[${index}]`));
  const result = vector(value, 2, label);
  result[0] = bounded(result[0], OPENARM_LAB_EQUIPMENT_LIMITS.cylinderRadiusM, `${label}[0]`);
  result[1] = bounded(result[1], OPENARM_LAB_EQUIPMENT_LIMITS.cylinderHeightM, `${label}[1]`);
  return result;
}
function assertPhysicalBounds(id, shape, positionM, sizeM) {
  const halfX = shape === 'box' ? sizeM[0] / 2 : sizeM[0];
  const halfY = shape === 'box' ? sizeM[1] / 2 : sizeM[0];
  const halfZ = shape === 'box' ? sizeM[2] / 2 : sizeM[1] / 2;
  const [x, y, z] = positionM;
  if (x - halfX < TABLETOP.xM[0] || x + halfX > TABLETOP.xM[1]) throw new RangeError(`OpenArm lab equipment ${id} extends beyond the physical tabletop in X`);
  if (y - halfY < TABLETOP.yM[0] || y + halfY > TABLETOP.yM[1]) throw new RangeError(`OpenArm lab equipment ${id} extends beyond the physical tabletop in Y`);
  if (z - halfZ < TABLETOP.zM - 1e-12) throw new RangeError(`OpenArm lab equipment ${id} intersects the physical tabletop; its bottom must be at or above z=${TABLETOP.zM}`);
  if (z + halfZ > WORKSPACE.zM[1] + 1e-12) throw new RangeError(`OpenArm lab equipment ${id} extends above the configured workcell ceiling z=${WORKSPACE.zM[1]}`);
}

export function normalizeOpenArmLabEquipment(items = []) {
  if (!Array.isArray(items)) throw new TypeError('OpenArm lab equipment must be an array');
  if (items.length > MAX_ITEMS) throw new RangeError(`OpenArm lab equipment supports at most ${MAX_ITEMS} items`);
  const ids = new Set();
  let freeCount = 0;
  const normalized = items.map((raw, index) => {
    if (!raw || typeof raw !== 'object' || Array.isArray(raw)) throw new TypeError(`OpenArm lab equipment item ${index} must be an object`);
    const allowed = new Set(['id', 'shape', 'mobility', 'positionM', 'sizeM', 'massKg']);
    for (const key of Object.keys(raw)) if (!allowed.has(key)) throw new TypeError(`Unexpected OpenArm lab equipment field: ${key}`);
    const id = String(raw.id || '');
    if (!ID_RE.test(id)) throw new TypeError(`OpenArm lab equipment id ${id || '<missing>'} must match ${ID_RE}`);
    if (ids.has(id)) throw new TypeError(`Duplicate OpenArm lab equipment id: ${id}`);
    ids.add(id);
    const shape = String(raw.shape || '');
    if (!SHAPES.has(shape)) throw new TypeError(`OpenArm lab equipment ${id} shape must be box or cylinder`);
    const mobility = String(raw.mobility || '');
    if (!MOBILITIES.has(mobility)) throw new TypeError(`OpenArm lab equipment ${id} mobility must be fixed or free`);
    const positionM = normalizePosition(raw.positionM, `${id}.positionM`);
    const sizeM = normalizeSize(shape, raw.sizeM, `${id}.sizeM`);
    assertPhysicalBounds(id, shape, positionM, sizeM);
    let massKg = null;
    if (mobility === 'free') {
      freeCount += 1;
      massKg = bounded(raw.massKg, OPENARM_LAB_EQUIPMENT_LIMITS.freeMassKg, `${id}.massKg`);
    } else if (raw.massKg != null) throw new TypeError(`Fixed OpenArm lab equipment ${id} must not declare massKg`);
    return Object.freeze({ id, shape, mobility, positionM: Object.freeze(positionM), sizeM: Object.freeze(sizeM), massKg });
  });
  if (freeCount > MAX_FREE_ITEMS) throw new RangeError(`OpenArm lab equipment supports at most ${MAX_FREE_ITEMS} free rigid bodies`);
  return Object.freeze(normalized);
}

export function openArmLabBodyName(id) { return `lab_${id}`; }
export function openArmLabGeomName(id) { return `lab_${id}_geom`; }
export function openArmLabFreeJointName(id) { return `lab_${id}_free`; }
