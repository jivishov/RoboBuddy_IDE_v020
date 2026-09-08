import { capabilityLabel, physicsCapabilityFor } from './capabilities.js';

export function applyPhysicsPreviewStatus(profileId) {
  const capability = physicsCapabilityFor(profileId);
  const backendBadge = document.getElementById('physicsBackendBadge');
  const modeChip = document.getElementById('modeChip');
  const simBadge = document.getElementById('simBadge');
  if (backendBadge) { backendBadge.textContent = capabilityLabel(capability); backendBadge.title = `${capability.capability} ${capability.limitations.join(' ')}`; }
  if (modeChip && capability.backend === 'browser-mujoco') modeChip.textContent = `${profileId === 'openarm' ? 'OPENARM V2' : 'SO-101'} PHYSICAL WORKSPACE · MUJOCO AUTHORITY`;
  if (simBadge && capability.backend === 'browser-mujoco') simBadge.textContent = 'MUJOCO RIGID-BODY BENCHMARK · ACTUAL CONTACT/GRAVITY · NOT HARDWARE CALIBRATION';
  return capability;
}
