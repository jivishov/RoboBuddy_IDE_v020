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
export function usableContact(c) { return Number.isFinite(c.normalForceN) && c.normalForceN > .01 && Number.isFinite(c.distanceM) && c.distanceM >= -.002; }
export function graspState(observation, side, bodyId) {
  const contacts = contactsForObject(observation, bodyId).filter(usableContact);
  const padForce = kind => contacts.filter(c => [c.geom1Name, c.geom2Name].some(n => n?.startsWith(`finger_${kind}_${side}_collision_`))).reduce((sum, c) => sum + c.normalForceN, 0);
  const innerForceN = padForce('inner'), outerForceN = padForce('outer');
  return { innerForceN, outerForceN, bilateralContact: innerForceN > .01 && outerForceN > .01, anyFingerContact: innerForceN > .01 || outerForceN > .01, source: 'MuJoCo contact forces; not hardware tactile measurements' };
}
export function worldPoint(body, local) {
  const [w, x, y, z] = body.quaternionWxyz;
  const [a, b, c] = local;
  const t = [2 * (y*c-z*b), 2 * (z*a-x*c), 2 * (x*b-y*a)];
  return [a+w*t[0]+y*t[2]-z*t[1], b+w*t[1]+z*t[0]-x*t[2], c+w*t[2]+x*t[1]-y*t[0]].map((v, i) => v + body.positionM[i]);
}
export function conditionMet(observation, condition) {
  const o = observation;
  if (condition.type === 'bilateral_grasp') return graspState(o, condition.side, condition.object_id).bilateralContact;
  if (condition.type === 'released') return !graspState(o, condition.side, condition.object_id).anyFingerContact;
  if (condition.type === 'supported') return contactsForObject(o, condition.object_id).some(c => usableContact(c) && [c.geom1Name, c.geom2Name].includes(condition.support_geom));
  if (condition.type === 'in_region') {
    const p = o.bodies?.[condition.object_id]?.positionM;
    return Boolean(p && p.every((v, i) => Math.abs(v - condition.center_m[i]) <= condition.half_extents_m[i]));
  }
  if (condition.type === 'equipment_joint') {
    const j = o.openarm?.equipmentJoints?.find(j => j.id === condition.joint_id);
    return Boolean(j && j.position >= condition.minimum && j.position <= condition.maximum);
  }
  if (condition.type === 'tool_reached') {
    const p = o.openarm?.pinchReferences?.[condition.side]?.positionM;
    return Boolean(p && Math.hypot(...p.map((v, i) => v - condition.position_m[i])) <= condition.tolerance_m);
  }
  return false;
}
