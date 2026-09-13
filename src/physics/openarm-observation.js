// Read-only interpretation of named simulator contacts. Not tactile sensor emulation.
export function objectGeometryIds(observation, bodyId) {
  if (bodyId === 'flask') return ['flask_body_geom', 'flask_shoulder_geom', 'flask_grip_geom'];
  if (bodyId === 'beaker') return ['beaker_grip_geom'];
  return (observation.openarm?.equipment || []).find(e => e.bodyId === bodyId)?.geometryIds || [];
}
export function contactsForObject(observation, bodyId) {
  const ids = new Set(objectGeometryIds(observation, bodyId));
  return (observation.contacts || []).filter(c => ids.has(c.geom1Name) || ids.has(c.geom2Name));
}
export function contactEvidenceComplete(observation) {
  // An empty or partial/unreadable contact list is not evidence of release.
  return observation.contactsReadable === true && Array.isArray(observation.contacts)
    && Number.isInteger(observation.contactCount) && observation.contactCount >= 0
    && observation.contactCount === observation.contacts.length;
}
export function usableContact(c) { return Number.isFinite(c.normalForceN) && c.normalForceN > .01 && Number.isFinite(c.distanceM) && c.distanceM >= -.002; }
function activeOrUncertainContact(c) {
  // Excessive penetration must REJECT a good-grasp claim, never erase contact
  // from the release test. Zero-force geometric touch and malformed telemetry
  // also conservatively prevent a release claim. A positive gap with zero load
  // is only a proximity contact and does not prevent release.
  return !Number.isFinite(c.distanceM) || !Number.isFinite(c.normalForceN)
    || c.distanceM <= 0 || c.normalForceN !== 0;
}
export function graspState(observation, side, bodyId) {
  const raw = contactsForObject(observation, bodyId);
  const complete = contactEvidenceComplete(observation);
  const contacts = complete ? raw.filter(usableContact) : [];
  const matches = (c, prefix) => [c.geom1Name, c.geom2Name].some(n => typeof n === 'string' && n.startsWith(prefix));
  const padForce = kind => contacts.filter(c => matches(c, `finger_${kind}_${side}_collision_`)).reduce((sum, c) => sum + c.normalForceN, 0);
  const innerForceN = padForce('inner'), outerForceN = padForce('outer');
  const anyFingerContact = raw.some(c => activeOrUncertainContact(c) && ['inner', 'outer'].some(kind => matches(c, `finger_${kind}_${side}_collision_`)));
  const anyPalmContact = raw.some(c => activeOrUncertainContact(c) && matches(c, `ee_base_link_${side}_collision_`));
  const excessivePenetrationSeen = raw.some(c => Number.isFinite(c.distanceM) && c.distanceM < -.002);
  return { innerForceN, outerForceN, bilateralContact: complete && !excessivePenetrationSeen && innerForceN > .01 && outerForceN > .01,
    anyFingerContact, anyPalmContact, anyGripperContact: anyFingerContact || anyPalmContact,
    contactsReadable: complete, excessivePenetrationSeen,
    source: 'MuJoCo contact forces; not hardware tactile measurements' };
}
export function worldPoint(body, local) {
  const [w, x, y, z] = body.quaternionWxyz;
  const [a, b, c] = local;
  const t = [2 * (y*c-z*b), 2 * (z*a-x*c), 2 * (x*b-y*a)];
  return [a+w*t[0]+y*t[2]-z*t[1], b+w*t[1]+z*t[0]-x*t[2], c+w*t[2]+x*t[1]-y*t[0]].map((v, i) => v + body.positionM[i]);
}
const finiteVec3 = p => Array.isArray(p) && p.length === 3 && p.every(Number.isFinite);
export function conditionMet(observation, condition) {
  const o = observation;
  if (condition.type === 'bilateral_grasp' || condition.type === 'released' || condition.type === 'supported') {
    if (!contactEvidenceComplete(o) || !o.bodies?.[condition.object_id] || !objectGeometryIds(o, condition.object_id).length) return false;
  }
  if (condition.type === 'bilateral_grasp') return graspState(o, condition.side, condition.object_id).bilateralContact;
  if (condition.type === 'released') return !graspState(o, condition.side, condition.object_id).anyGripperContact;
  // This predicate establishes force-bearing contact with the named geometry;
  // it is not, alone, a certificate of stable placement or upward load support.
  if (condition.type === 'supported') return contactsForObject(o, condition.object_id).some(c => usableContact(c) && [c.geom1Name, c.geom2Name].includes(condition.support_geom));
  if (condition.type === 'in_region') {
    const p = o.bodies?.[condition.object_id]?.positionM;
    return Boolean(finiteVec3(p) && p.every((v, i) => Math.abs(v - condition.center_m[i]) <= condition.half_extents_m[i]));
  }
  if (condition.type === 'equipment_joint') {
    const j = o.openarm?.equipmentJoints?.find(j => j.id === condition.joint_id);
    return Boolean(j && Number.isFinite(j.position) && j.position >= condition.minimum && j.position <= condition.maximum);
  }
  if (condition.type === 'tool_reached') {
    const p = o.openarm?.pinchReferences?.[condition.side]?.positionM;
    return Boolean(finiteVec3(p) && Math.hypot(...p.map((v, i) => v - condition.position_m[i])) <= condition.tolerance_m);
  }
  return false;
}
