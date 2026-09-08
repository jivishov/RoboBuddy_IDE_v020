import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { CanonicalRobotRig } from '../canonical-rig.js';
import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import { OPENARM_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';
import { OPENARM_PHASE5A_SCENE } from './openarm-scene.js';
import { OpenArmPhase5AEvaluator, hasOpenArmObjectGripperContact, hasOpenArmObjectSupportContact } from './openarm-task-evaluator.js';

const RAD_TO_DEG = 180 / Math.PI;
const STEP_ALIGNMENT_TOLERANCE_SECONDS = 1e-9;
const MAX_WEBMCP_ADVANCE_SECONDS = 2;
const PRESENTATION_GROUND_COLOR = 0x687378;

function toThreePosition(positionM = [0, 0, 0]) {
  return new THREE.Vector3(Number(positionM[0]) * 1000, Number(positionM[2]) * 1000, -Number(positionM[1]) * 1000);
}
function toThreeQuaternion(quaternionWxyz = [1, 0, 0, 0]) {
  const [w, x, y, z] = quaternionWxyz.map(Number);
  const physical = new THREE.Quaternion(x, y, z, w).normalize();
  const basis = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
  const physicalRotation = new THREE.Matrix4().makeRotationFromQuaternion(physical);
  return new THREE.Quaternion().setFromRotationMatrix(basis.clone().multiply(physicalRotation).multiply(basis.clone().invert())).normalize();
}

function allowedPhysicalTargets(targetsRad) {
  if (!targetsRad || typeof targetsRad !== 'object' || Array.isArray(targetsRad)) throw new TypeError('targetsRad must be an object');
  const entries = Object.entries(targetsRad);
  if (!entries.length) throw new TypeError('targetsRad must contain at least one OpenArm V2 actuator target');
  const actuatorByJoint = new Map(OPENARM_PHASE5A_MODEL_PACKAGE.actuators.map((actuator) => [actuator.jointId, actuator]));
  const jointById = new Map(OPENARM_PHASE5A_MODEL_PACKAGE.joints.map((joint) => [joint.id, joint]));
  const clean = {};
  for (const [jointId, raw] of entries) {
    const actuator = actuatorByJoint.get(jointId);
    const joint = jointById.get(jointId);
    if (!actuator || !joint) throw new Error(`Unknown or passive physical OpenArm V2 joint ${jointId}`);
    const value = Number(raw);
    if (!Number.isFinite(value)) throw new TypeError(`Physical OpenArm V2 target ${jointId} must be finite radians`);
    const minimum = Math.max(Number(actuator.controlRangeRad[0]), Number(joint.rangeRad[0]));
    const maximum = Math.min(Number(actuator.controlRangeRad[1]), Number(joint.rangeRad[1]));
    if (value < minimum || value > maximum) throw new RangeError(`${jointId}=${value} rad is outside ${minimum}..${maximum} rad`);
    clean[jointId] = value;
  }
  return clean;
}

function boxVisual(name, centerM, halfSizeM, color) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(halfSizeM[0] * 2000, halfSizeM[2] * 2000, halfSizeM[1] * 2000),
    new THREE.MeshStandardMaterial({ color, roughness: 0.72, metalness: 0.05 }),
  );
  mesh.name = name;
  mesh.position.copy(toThreePosition(centerM));
  mesh.receiveShadow = true;
  return mesh;
}

export class OpenArmPhysicalSimulator {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xb9c1c4);
    this.camera = new THREE.PerspectiveCamera(43, 1, 1, 5000);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.35));
    const key = new THREE.DirectionalLight(0xffffff, 2.0); key.position.set(500, 1500, 750); key.castShadow = true; this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x93c5fd, 0.4); fill.position.set(-500, 900, -600); this.scene.add(fill);
    const grid = new THREE.GridHelper(2200, 44, 0x334155, 0x4b5563); grid.userData.presentationOnly = true; this.scene.add(grid);
    this.workcellRoot = new THREE.Group(); this.workcellRoot.name = 'openarm-phase5a-workcell-presentation'; this.scene.add(this.workcellRoot);
    this.rig = null;
    this.objectMeshes = new Map();
    this.targetMarkers = [];
    this.session = null;
    this.unsubscribeSession = null;
    this.evaluator = null;
    this.lastObservation = null;
    this.ready = false;
    this.disposed = false;
    this.highContrast = true;
    this.sessionSequence = 0;
    this.resize();
  }

  async setScenario(profileId, scenario) {
    if (profileId !== 'openarm') throw new Error('OpenArm physical simulator only accepts the openarm profile');
    if (scenario?.simulationMode !== 'physical_mujoco' || scenario?.physicalSceneId !== OPENARM_PHASE5A_SCENE.id) throw new Error('OpenArm physical simulator requires the Phase 5A physical workspace');
    await this.#disposeSession();
    this.#clearPresentation();
    this.rig = await CanonicalRobotRig.load('openarm');
    // Canonical low-stand mesh mounts shoulders at 0.550 m. Source V2 cell home
    // mounts them at 1.340 m and x=0.185 m; this is presentation alignment only.
    this.rig.root.position.set(185, 790, 0);
    this.scene.add(this.rig.root);
    this.#buildWorkcell();
    this.evaluator = new OpenArmPhase5AEvaluator();
    await this.#createSession();
    this.ready = true;
    this.canvas.dataset.simulatorBackend = 'browser-mujoco';
    this.canvas.dataset.simulationAuthority = 'physics-session';
    this.canvas.dataset.physicalSceneId = OPENARM_PHASE5A_SCENE.id;
    this.canvas.dataset.modelPackageId = OPENARM_PHASE5A_SCENE.modelPackage;
    this.canvas.dataset.presentationGroundColor = '#687378';
    this.setHighContrastScene(this.highContrast);
    this.fit();
    return true;
  }

  async reset() {
    this.#assertNotDisposed();
    this.ready = false;
    this.evaluator = new OpenArmPhase5AEvaluator();
    if (!this.session) await this.#createSession();
    else {
      let diagnostics = null;
      try { diagnostics = await this.session.getDiagnostics(); } catch {}
      if (!diagnostics?.loaded) { await this.#disposeSession(); await this.#createSession(); }
      else await this.session.reset({ reason: 'explicit-user-reset' });
    }
    this.ready = true;
    return true;
  }

  renderFrame() { if (!this.disposed) { this.controls.update(); this.renderer.render(this.scene, this.camera); } }
  resize() {
    if (this.disposed) return false;
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 640);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 480);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.floor(width * dpr) || this.canvas.height !== Math.floor(height * dpr)) this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height; this.camera.updateProjectionMatrix(); return true;
  }
  fit() {
    this.controls.target.set(430, 1070, 0);
    this.camera.position.set(1250, 1450, 1250);
    this.camera.near = 1; this.camera.far = 6000; this.camera.updateProjectionMatrix(); this.controls.update();
    this.canvas.dataset.cameraView = 'front';
    return true;
  }
  setHighContrastScene(value) {
    this.highContrast = Boolean(value);
    this.targetMarkers.forEach((mesh) => { mesh.visible = this.highContrast; });
    this.canvas.dataset.highContrastScene = String(this.highContrast);
    this.canvas.dataset.highContrastPerimeterCount = this.highContrast ? String(this.targetMarkers.length) : '0';
    this.canvas.dataset.presentationGroundColor = '#687378';
    return this.highContrast;
  }
  isHighContrastSceneEnabled() { return this.highContrast; }
  isReady() { return Boolean(this.ready && this.session && this.lastObservation && this.session.sceneRevision && this.session.robotId); }
  getPhysicalSession() { return this.session; }
  getPhysicalAuthorityToken() {
    if (!this.isReady()) return null;
    return Object.freeze({ sessionId: this.session.sessionId, epoch: this.session.epoch, sceneRevision: this.session.sceneRevision, robotId: this.session.robotId, simulationTimeSeconds: this.lastObservation.simulationTimeSeconds });
  }
  getTaskEvaluation() { return this.evaluator?.snapshot?.() || null; }
  getTelemetry() {
    const observation = this.lastObservation;
    if (!observation) return {};
    const out = { simulation_time_s: Number(observation.simulationTimeSeconds) };
    for (const [jointId, state] of Object.entries(observation.joints || {})) out[`${jointId}_rad`] = Number(state.positionRad);
    for (const objectId of ['phase5a_flask', 'phase5a_beaker']) {
      const body = observation.bodies?.[objectId];
      if (body?.positionM) ['x','y','z'].forEach((axis, index) => { out[`${objectId}_${axis}_m`] = Number(body.positionM[index]); });
      if (body?.linearVelocityMPerS) out[`${objectId}_linear_speed_m_s`] = Math.hypot(...body.linearVelocityMPerS.map(Number));
    }
    return out;
  }
  getContacts() {
    const observation = this.lastObservation;
    const evaluation = this.getTaskEvaluation();
    if (!observation || !evaluation) return {};
    return {
      contact_count: Number(observation.contactCount || 0),
      flask_gripper_contact: hasOpenArmObjectGripperContact(observation, 'flask'),
      flask_hotplate_contact: hasOpenArmObjectSupportContact(observation, 'flask'),
      beaker_gripper_contact: hasOpenArmObjectGripperContact(observation, 'beaker'),
      beaker_gauze_contact: hasOpenArmObjectSupportContact(observation, 'beaker'),
      order_violation: evaluation.orderViolation,
      task_success: evaluation.success,
    };
  }
  getState() { return this.lastObservation ? { observation: structuredClone(this.lastObservation), evaluation: this.getTaskEvaluation(), authority: this.getPhysicalAuthorityToken() } : null; }
  async applyAction() { throw new Error('Legacy OpenArm .pos replay is disabled in the physical workspace. Use robobuddy.sim.v1 radians or the bounded OpenArm physical WebMCP schema.'); }
  async applyPhysicalTargets(targetsRad, { maxSteps = 5000, advanceSeconds = 0 } = {}) {
    this.#assertReady();
    const cleanTargets = allowedPhysicalTargets(targetsRad);
    if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > 10000) throw new RangeError('maxSteps must be an integer from 1 to 10000');
    const advance = Number(advanceSeconds);
    if (!Number.isFinite(advance) || advance < 0 || advance > MAX_WEBMCP_ADVANCE_SECONDS) throw new RangeError(`advanceSeconds must be between 0 and ${MAX_WEBMCP_ADVANCE_SECONDS}`);
    const accepted = await this.session.sendCommand({ type: 'set_joint_targets', targetsRad: cleanTargets }, { maxSteps });
    let observation = accepted.observation;
    if (advance > 0) observation = await this.advanceTime(advance);
    return { schemaVersion: 'robobuddy.openarm.physical.v1', status: accepted.status, commandId: accepted.commandId, acceptedTargetsRad: cleanTargets, observation: structuredClone(observation), taskEvaluation: this.getTaskEvaluation() };
  }
  async advanceTime(seconds) {
    this.#assertReady();
    const duration = Number(seconds);
    if (!Number.isFinite(duration) || duration < 0) throw new RangeError('Physical advance duration must be finite and non-negative');
    if (duration === 0) return this.session.getObservation({ view: 'ground_truth' });
    const dt = Number(this.lastObservation?.engine?.timestepSeconds);
    const steps = Math.round(duration / dt);
    if (steps < 1 || Math.abs(duration - steps * dt) > STEP_ALIGNMENT_TOLERANCE_SECONDS) throw new RangeError(`Physical advance duration ${duration}s must align to the ${dt}s MuJoCo timestep`);
    return this.session.advanceSteps(steps);
  }
  pause() { return this.session?.pause?.() ?? false; }
  resume() { return this.session?.resume?.() ?? false; }
  async stop() {
    if (!this.session) return false;
    this.ready = false;
    try { const diagnostics = await this.session.getDiagnostics(); if (!diagnostics?.loaded) return false; await this.session.cancelRun('human-stop'); return true; } catch { return false; }
  }
  dispose() { if (!this.disposed) { this.disposed = true; this.ready = false; void this.#disposeSession(); this.#clearPresentation(); this.controls?.dispose?.(); this.renderer?.dispose?.(); } }

  async #createSession() {
    const session = new PhysicsSession(new BrowserMuJoCoBackend(), { sessionId: `ide-openarm-${++this.sessionSequence}` });
    this.session = session;
    this.unsubscribeSession = session.subscribe(({ observation }) => this.#consumeObservation(observation));
    await session.loadScene(structuredClone(OPENARM_PHASE5A_SCENE));
  }
  async #disposeSession() { this.unsubscribeSession?.(); this.unsubscribeSession = null; if (this.session) this.session.dispose(); this.session = null; this.lastObservation = null; }
  #consumeObservation(observation) {
    if (!observation || this.disposed) return;
    this.lastObservation = structuredClone(observation);
    this.evaluator?.observe(observation);
    this.#applyRobotObservation(observation);
    for (const objectId of ['phase5a_flask', 'phase5a_beaker']) this.#applyObjectObservation(objectId, observation);
    this.canvas.dataset.simulationClockS = String(Number(observation.simulationTimeSeconds || 0));
    const evaluation = this.getTaskEvaluation();
    this.canvas.dataset.physicalTaskSuccess = String(Boolean(evaluation?.success));
    this.canvas.dataset.physicalTaskContact = String(Boolean(evaluation?.flask?.contactSeen || evaluation?.beaker?.contactSeen));
    this.canvas.dataset.physicalTaskLift = String(Boolean(evaluation?.flask?.liftSeen && evaluation?.beaker?.liftSeen));
    this.canvas.dataset.physicalTaskCarry = String(Boolean(evaluation?.flask?.carrySeen && evaluation?.beaker?.carrySeen));
    this.canvas.dataset.physicalTaskRelease = String(Boolean(evaluation?.flask?.releaseSeen && evaluation?.beaker?.releaseSeen));
    this.canvas.dataset.physicalTaskSettled = String(Boolean(evaluation?.flask?.settleSeen && evaluation?.beaker?.settleSeen));
  }
  #applyRobotObservation(observation) {
    if (!this.rig) return;
    const state = {};
    for (const side of ['left', 'right']) {
      for (let index = 1; index <= 7; index += 1) {
        const q = Number(observation.joints?.[`openarm_${side}_joint${index}`]?.positionRad);
        if (Number.isFinite(q)) state[`${side}_joint_${index}.pos`] = q * RAD_TO_DEG;
      }
      const q = Number(observation.joints?.[`openarm_${side}_finger_joint1`]?.positionRad);
      if (Number.isFinite(q)) {
        const openDeg = side === 'left' ? q * RAD_TO_DEG : -q * RAD_TO_DEG;
        state[`${side}_gripper.pos`] = -Math.max(0, Math.min(45, openDeg)) * (65 / 45);
      }
    }
    this.rig.applyPhysicalState(state);
  }
  #applyObjectObservation(objectId, observation) {
    const mesh = this.objectMeshes.get(objectId);
    const body = observation.bodies?.[objectId];
    if (!mesh || !body?.positionM) return;
    mesh.position.copy(toThreePosition(body.positionM));
    if (body.quaternionWxyz) mesh.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));
  }
  #buildWorkcell() {
    const support = new THREE.MeshStandardMaterial({ color: PRESENTATION_GROUND_COLOR, roughness: 0.76, metalness: 0.03 });
    const table = boxVisual('visual-cell-table', [0.41, 0, 1.000], [0.41, 0.55, 0.005], PRESENTATION_GROUND_COLOR); this.workcellRoot.add(table);
    const hotplate = boxVisual('visual-hotplate', [0.4318, 0.2397, 1.0165], [0.060, 0.055, 0.0115], 0x33383f); this.workcellRoot.add(hotplate);
    const gauze = new THREE.Mesh(new THREE.CylinderGeometry(35, 35, 1, 36), support); gauze.name = 'visual-wire-gauze'; gauze.position.copy(toThreePosition([0.4771, -0.2397, 1.0745])); this.workcellRoot.add(gauze);
    const post = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 68, 16), support); post.position.copy(toThreePosition([0.535, -0.2397, 1.045])); this.workcellRoot.add(post);
    const radius = 10; const height = OPENARM_PHASE5A_MODEL_PACKAGE.benchmark.objectHalfHeightM * 2000;
    for (const [id, color] of [['phase5a_flask', 0x3b82f6], ['phase5a_beaker', 0x67e8f9]]) {
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, height, 32), new THREE.MeshStandardMaterial({ color, roughness: 0.35, metalness: 0.02, transparent: true, opacity: 0.82 }));
      mesh.name = `visual-${id}`; mesh.castShadow = true; mesh.receiveShadow = true; this.objectMeshes.set(id, mesh); this.workcellRoot.add(mesh);
    }
    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.35, depthWrite: false });
    for (const def of [OPENARM_PHASE5A_MODEL_PACKAGE.benchmark.flask, OPENARM_PHASE5A_MODEL_PACKAGE.benchmark.beaker]) {
      const marker = new THREE.Mesh(new THREE.CylinderGeometry(30, 30, 1, 36), markerMaterial.clone());
      marker.position.copy(toThreePosition([def.targetCenterXYM[0], def.targetCenterXYM[1], def.supportTopZM + 0.0005]));
      marker.userData.presentationOnly = true; this.targetMarkers.push(marker); this.workcellRoot.add(marker);
    }
  }
  #clearPresentation() {
    if (this.rig) { this.scene.remove(this.rig.root); this.rig.dispose(); this.rig = null; }
    for (const child of [...this.workcellRoot.children]) {
      child.traverse?.((node) => { if (!node.isMesh) return; node.geometry?.dispose?.(); if (Array.isArray(node.material)) node.material.forEach((m) => m?.dispose?.()); else node.material?.dispose?.(); });
      this.workcellRoot.remove(child);
    }
    this.objectMeshes.clear(); this.targetMarkers = [];
  }
  #assertReady() { this.#assertNotDisposed(); if (!this.isReady()) throw new Error('OpenArm physical session is not ready'); }
  #assertNotDisposed() { if (this.disposed) throw new Error('OpenArm physical simulator is disposed'); }
}
