import { capabilityLabel, physicsCapabilityFor } from './capabilities.js';

export function applyPhysicsPreviewStatus(profileId) {
  const capability = physicsCapabilityFor(profileId);
  const backendBadge = document.getElementById('physicsBackendBadge');
  const modeChip = document.getElementById('modeChip');
  const simBadge = document.getElementById('simBadge');
  if (backendBadge) {
    backendBadge.textContent = capabilityLabel(capability);
    backendBadge.title = `${capability.capability} ${capability.limitations.join(' ')}`;
  }
  if (modeChip && capability.backend === 'legacy') {
    modeChip.textContent = 'PHYSICS MIGRATION · LEGACY BACKEND ACTIVE';
  }
  if (simBadge && capability.backend === 'legacy') {
    simBadge.textContent = 'PHYSICS PREVIEW · LEGACY MODEL ACTIVE · NOT HARDWARE VALIDATION';
  }
  return capability;
}
