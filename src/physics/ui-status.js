import { capabilityLabel, physicsCapabilityFor } from './capabilities.js';

export function applyPhysicsPreviewStatus(profileId) {
  const capability = physicsCapabilityFor(profileId);
  const backendBadge = document.getElementById('physicsBackendBadge');
  const modeChip = document.getElementById('modeChip');
  const simBadge = document.getElementById('simBadge');
  const fidelityText = document.getElementById('fidelityText');
  if (backendBadge) {
    backendBadge.textContent = capabilityLabel(capability);
    backendBadge.title = `${capability.capability} ${capability.limitations.join(' ')}`;
  }
  if (modeChip && capability.backend === 'browser-mujoco') {
    modeChip.textContent = profileId === 'openarm'
      ? 'OPENARM V2 PHYSICAL WORKSPACE · MUJOCO AUTHORITY'
      : 'SO-101 PHYSICAL WORKSPACE · MUJOCO AUTHORITY';
  }
  if (simBadge && capability.backend === 'browser-mujoco') {
    simBadge.textContent = profileId === 'openarm'
      ? 'MUJOCO V2 BIMANUAL DRY-STACK · SHARED FREE BODIES · NOT HARDWARE CALIBRATION'
      : 'MUJOCO RIGID-BODY BENCHMARK · ACTUAL CONTACT/GRAVITY · NOT HARDWARE CALIBRATION';
  }
  if (fidelityText && capability.backend === 'browser-mujoco' && profileId === 'openarm') {
    fidelityText.textContent = 'OpenArm V2 uses one authoritative browser MuJoCo PhysicsSession. Both arms, the free flask and beaker, fixtures, live Python, WebMCP and task evaluation consume that state. Only the source mechanical finger couplings remain constrained; task objects are never attached, welded, snapped, teleported or moved by renderer state. Collision proxies and dry workcell parameters are simulator estimates; hardware validation remains pending.';
  }
  return capability;
}
