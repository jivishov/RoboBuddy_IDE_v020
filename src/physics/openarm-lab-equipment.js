export * from './openarm-lab-equipment-base.js';
import { validateOpenArmLabEquipment as validateBase } from './openarm-lab-equipment-base.js';

// Ordinary authored laboratory equipment is free-standing by default. The bench is the only
// builder-owned world-fixed boundary condition by default. A caller may explicitly request
// dynamic:false for a receiver, tray, rack or obstacle when modelling an installed/bolted fixture
// or an approved fixed idealization. supported_by metadata is descriptive only and never welds.
const FREE_BY_DEFAULT = new Set(['tray', 'rack', 'receiver', 'obstacle']);

export function validateOpenArmLabEquipment(items) {
  if (!Array.isArray(items)) return validateBase(items);
  const desiredDynamics = items.map(item => {
    if (item && FREE_BY_DEFAULT.has(item.kind)) return item.dynamic ?? true;
    return item?.dynamic;
  });
  // The base validator historically rejects dynamic ancillary equipment. Validate all of its
  // shape/bounds/mass invariants with a temporary fixed flag, then restore the explicit/default
  // free-body semantics. This wrapper is the Lab Builder public contract.
  const baseInput = items.map(item => {
    if (!item || !FREE_BY_DEFAULT.has(item.kind)) return item;
    return { ...item, dynamic: false };
  });
  const normalized = validateBase(baseInput);
  return normalized.map((item, index) => {
    if (!FREE_BY_DEFAULT.has(item.kind)) return item;
    return { ...item, dynamic: Boolean(desiredDynamics[index]) };
  });
}
