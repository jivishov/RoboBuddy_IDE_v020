export * from './openarm-lab-equipment-base.js';
import { validateOpenArmLabEquipment as validateBase } from './openarm-lab-equipment-base.js';

// Trays and racks are ordinary free-standing bodies unless the author explicitly declares
// dynamic:false as a fixed idealization. supported_by metadata never creates a weld.
export function validateOpenArmLabEquipment(items) {
  if (!Array.isArray(items)) return validateBase(items);
  const desiredDynamics = items.map(item => {
    if (item && ['tray', 'rack'].includes(item.kind)) return item.dynamic ?? true;
    return item?.dynamic;
  });
  const baseInput = items.map(item => {
    if (!item || !['tray', 'rack'].includes(item.kind)) return item;
    return { ...item, dynamic: false };
  });
  const normalized = validateBase(baseInput);
  return normalized.map((item, index) => {
    if (!['tray', 'rack'].includes(item.kind)) return item;
    return { ...item, dynamic: Boolean(desiredDynamics[index]) };
  });
}
