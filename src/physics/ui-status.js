import { capabilityLabel, physicsCapabilityFor } from './capabilities.js';

const PHYSICAL_MODE_CHIP = Object.freeze({
  openarm: 'OPENARM V2 PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
  lekiwi: 'LEKIWI V1 PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
  so101: 'SO-101 PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
});

const PHYSICAL_SIM_BADGE = Object.freeze({
  openarm: 'MUJOCO V2 BIMANUAL DRY-STACK · SHARED FREE BODIES · NOT HARDWARE CALIBRATION',
  lekiwi: 'MUJOCO HOLONOMIC BASE · PASSIVE OMNI ROLLERS · FREE PAYLOAD · NOT HARDWARE CALIBRATION',
  so101: 'MUJOCO RIGID-BODY BENCHMARK · ACTUAL CONTACT/GRAVITY · NOT HARDWARE CALIBRATION',
});

export function applyPhysicsPreviewStatus(profileId, { physical = true } = {}) {
  const capability = physicsCapabilityFor(profileId, { physical });
  const backendBadge = document.getElementById('physicsBackendBadge');
  const modeChip = document.getElementById('modeChip');
  const simBadge = document.getElementById('simBadge');
  const fidelityText = document.getElementById('fidelityText');
  if (backendBadge) {
    backendBadge.textContent = capabilityLabel(capability);
    backendBadge.title = `${capability.capability} ${capability.limitations.join(' ')}`;
  }
  if (modeChip && capability.backend === 'browser-mujoco' && PHYSICAL_MODE_CHIP[profileId]) {
    modeChip.textContent = PHYSICAL_MODE_CHIP[profileId];
  }
  if (simBadge && capability.backend === 'browser-mujoco' && PHYSICAL_SIM_BADGE[profileId]) {
    simBadge.textContent = PHYSICAL_SIM_BADGE[profileId];
  }
  if (fidelityText && capability.backend === 'browser-mujoco' && profileId === 'openarm') {
    fidelityText.textContent = 'OpenArm V2 uses one authoritative browser MuJoCo PhysicsSession. Both arms, the free flask and beaker, fixtures, live Python, WebMCP and task evaluation consume that state. Only the source mechanical finger couplings remain constrained; task objects are never attached, welded, snapped, teleported or moved by renderer state. Collision proxies and dry workcell parameters are simulator estimates; hardware validation remains pending.';
  }
  return capability;
}
