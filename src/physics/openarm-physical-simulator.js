import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { CanonicalRobotRig, canonicalVisualProvenance } from '../canonical-rig.js';
import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import { OPENARM_V2_PHASE5A_MODEL_PACKAGE } from './openarm-model-package.js';
import { OPENARM_V2_PHASE5A_SCENE } from './openarm-scene.js';
import { OpenArmBimanualStackEvaluator } from './openarm-task-evaluator.js';

const MAX_WEBMCP_ADVANCE_SECONDS = 2;
const STEP_ALIGNMENT_TOLERANCE_SECONDS = 1e-9;
// Declared observation cadence, in authoritative MuJoCo steps. The narrowest causal event
// this task depends on is the beaker's intended gauze support contact while the vessel is
// still bilaterally pinched; native 1 ms evidence measures that overlap at only 3-4 ms, so a
// sampling period of two physics steps (2 ms at the pinned 0.001 s timestep) is guaranteed to
// land inside any window of two or more steps. Sampling happens inside the MuJoCo worker, so
// this resolution costs one cross-thread request per advance rather than one per sample.
const OPENARM_OBSERVATION_BATCH_STEPS = 2;
const PRESENTATION_GROUND_COLOR = 0x687378;
const CANONICAL_OPENARM_MOUNT_TRANSLATION_MM = Object.freeze([185, 790, 0]);
const NONPHYSICAL_CANONICAL_PARTS = new Set([
  'turntable_pedestal',
  'turntable_bearing',
  'turntable_disc',
  'turntable_heading',
  'openarm_body_link0_low_stand',
]);
const PHYSICAL_GRIPPER_MAX_RAD = Math.PI / 4;

function toThreePosition(positionM = [0, 0, 0]) {
  return new THREE.Vector3(Number(positionM[0]) * 1000, Number(positionM[2]) * 1000, -Number(positionM[1]) * 1000);
}
function toThreeQuaternion(quaternionWxyz = [1, 0, 0, 0]) {
  const [w, x, y, z] = quaternionWxyz.map(Number);
  const physical = new THREE.Quaternion(x, y, z, w).normalize();
  const basis = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
  const inverse = basis.clone().invert();
  const physicalRotation = new THREE.Matrix4().makeRotationFromQuaternion(physical);
  const converted = basis.clone().multiply(physicalRotation).multiply(inverse);
  return new THREE.Quaternion().setFromRotationMatrix(converted).normalize();
}
function finiteTargetMap(targetsRad) {
  if (!targetsRad || typeof targetsRad !== 'object' || Array.isArray(targetsRad)) throw new TypeError('targetsRad must be an object');
  const entries = Object.entries(targetsRad);
  if (!entries.length) throw new TypeError('targetsRad must contain at least one joint target');
  const allowed = new Set(OPENARM_V2_PHASE5A_MODEL_PACKAGE.actuators.map((item) => item.jointId));
  const clean = {};
  for (const [jointId, raw] of entries) {
    if (!allowed.has(jointId)) throw new Error(`Unknown physical OpenArm V2 joint ${jointId}`);
    const value = Number(raw);
    if (!Number.isFinite(value)) throw new TypeError(`Physical OpenArm target ${jointId} must be finite radians`);
    clean[jointId] = value;
  }
  return clean;
}
function box(widthM, heightM, depthM, material) {
  return new THREE.Mesh(new THREE.BoxGeometry(widthM * 1000, heightM * 1000, depthM * 1000), material);
}
function cylinder(radiusM, heightM, material) {
  return new THREE.Mesh(new THREE.CylinderGeometry(radiusM * 1000, radiusM * 1000, heightM * 1000, 32), material);
}
function canonicalStateFromObservation(observation) {
  const state = {};
  for (const side of ['left', 'right']) {
    for (let index = 1; index <= 7; index += 1) {
      const joint = observation?.joints?.[`openarm_${side}_joint${index}`];
      const positionRad = Number(joint?.positionRad);
      if (Number.isFinite(positionRad)) state[`${side}_joint_${index}.pos`] = THREE.MathUtils.radToDeg(positionRad);
    }
    const fingerRad = Math.abs(Number(observation?.joints?.[`openarm_${side}_finger_joint1`]?.positionRad));
    if (Number.isFinite(fingerRad)) {
      // Canonical presentation adapter uses the legacy public gripper scale only as
      // a visual conversion. It is not a physical command or hardware claim.
      state[`${side}_gripper.pos`] = -Math.min(65, (fingerRad / PHYSICAL_GRIPPER_MAX_RAD) * 65);
    }
  }
  return state;
}

export class OpenArmPhysicalSimulator {
  constructor(canvas) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xb9c1c4);
    this.camera = new THREE.PerspectiveCamera(42, 1, 1, 6000);
    this.camera.position.set(1900, 1500, 0);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(420, 1160, 0);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.35));
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.position.set(650, 1500, 850);
    key.castShadow = true;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x93c5fd, 0.4);
    fill.position.set(-500, 900, -900);
    this.scene.add(fill);
    const grid = new THREE.GridHelper(1800, 36, 0x334155, 0x4b5563);
    grid.userData.presentationOnly = true;
    this.scene.add(grid);

    this.workcellRoot = new THREE.Group();
    this.workcellRoot.name = 'openarm-v2-physical-workcell-presentation';
    this.scene.add(this.workcellRoot);
    this.robotRoot = new THREE.Group();
    this.robotRoot.name = 'openarm-v2-source-aligned-canonical-arm-presentation';
    this.scene.add(this.robotRoot);
    this.canonicalRig = null;
    this.hiddenCanonicalParts = [];
    this.objectMeshes = new Map();
    this.targetMarkers = [];
    this.session = null;
    this.unsubscribeSession = null;
    this.evaluator = null;
    this.lastObservation = null;
    this.presentationDirty = false;
    this.ready = false;
    this.disposed = false;
    this.highContrast = true;
    this.sessionSequence = 0;
    this.#buildPresentation();
    this.setHighContrastScene(true);
    this.resize();
  }

  async setScenario(profileId, scenario) {
    if (profileId !== 'openarm') throw new Error('OpenArm physical simulator only accepts the openarm profile');
    if (scenario?.simulationMode !== 'physical_mujoco' || scenario?.physicalSceneId !== OPENARM_V2_PHASE5A_SCENE.id) {
      throw new Error('OpenArm physical simulator requires the Phase 5A V2 bimanual scene');
    }
    await this.#ensureCanonicalPresentation();
    await this.#disposeSession();
    this.evaluator = new OpenArmBimanualStackEvaluator();
    await this.#createSession();
    this.ready = true;
    this.canvas.dataset.simulatorBackend = 'browser-mujoco';
    this.canvas.dataset.simulationAuthority = 'physics-session';
    this.canvas.dataset.physicalSceneId = OPENARM_V2_PHASE5A_SCENE.id;
    this.canvas.dataset.modelPackageId = OPENARM_V2_PHASE5A_SCENE.modelPackage;
    this.canvas.dataset.presentationGroundColor = '#687378';
    this.canvas.dataset.openarmVisualSource = 'canonical-v2-arm-mesh-source-aligned';
    this.canvas.dataset.openarmLegacyBaseYawRendered = 'false';
    this.fit();
    return true;
  }

  async reset() {
    this.#assertNotDisposed();
    this.ready = false;
    this.evaluator = new OpenArmBimanualStackEvaluator();
    if (!this.session) await this.#createSession();
    else {
      let diagnostics = null;
      try { diagnostics = await this.session.getDiagnostics(); } catch {}
      if (!diagnostics?.loaded) {
        await this.#disposeSession();
        await this.#createSession();
      } else await this.session.reset({ reason: 'explicit-user-reset' });
    }
    this.ready = true;
    return true;
  }

  renderFrame() {
    if (this.disposed) return;
    if (this.presentationDirty && this.lastObservation) {
      this.#applyObservation(this.lastObservation);
      this.presentationDirty = false;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
  resize() {
    if (this.disposed) return false;
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 640);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 480);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.floor(width * dpr) || this.canvas.height !== Math.floor(height * dpr)) this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    return true;
  }
  fit() {
    this.controls.target.set(420, 1160, 0);
    this.camera.position.set(1900, 1500, 0);
    this.camera.near = 1;
    this.camera.far = 6000;
    this.camera.updateProjectionMatrix();
    this.controls.update();
    return true;
  }
  setHighContrastScene(value) {
    this.highContrast = Boolean(value);
    for (const marker of this.targetMarkers) marker.visible = this.highContrast;
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
  getPresentationAudit() {
    const mountPositionsMm = {};
    for (const side of ['left', 'right']) {
      const position = this.canonicalRig?.getWorldPosition?.(`${side}_mount`);
      mountPositionsMm[side] = position ? position.toArray() : null;
    }
    return Object.freeze({
      source: canonicalVisualProvenance('openarm'),
      physicalAuthority: 'MuJoCo PhysicsSession only',
      jointPresentationSource: 'observed MuJoCo joint positions',
      mountTranslationMm: [...CANONICAL_OPENARM_MOUNT_TRANSLATION_MM],
      canonicalMountPositionsMm: mountPositionsMm,
      hiddenNonphysicalParts: [...this.hiddenCanonicalParts],
      legacyBaseYawControlled: false,
      legacyBaseYawRendered: false,
      observationBatchSteps: OPENARM_OBSERVATION_BATCH_STEPS,
      observationPeriodSeconds: OPENARM_OBSERVATION_BATCH_STEPS * Number(this.lastObservation?.engine?.timestepSeconds || 0.001),
    });
  }
  getTelemetry() {
    const observation = this.lastObservation;
    if (!observation) return {};
    const out = { simulation_time_s: Number(observation.simulationTimeSeconds) };
    for (const [jointId, state] of Object.entries(observation.joints || {})) out[`${jointId}_rad`] = Number(state.positionRad);
    for (const objectId of ['flask', 'beaker']) {
      const body = observation.bodies?.[objectId];
      if (!body?.positionM) continue;
      out[`${objectId}_x_m`] = Number(body.positionM[0]);
      out[`${objectId}_y_m`] = Number(body.positionM[1]);
      out[`${objectId}_z_m`] = Number(body.positionM[2]);
      out[`${objectId}_linear_speed_m_s`] = Math.hypot(...(body.linearVelocityMS || [0, 0, 0]).map(Number));
    }
    return out;
  }
  getContacts() {
    const observation = this.lastObservation;
    const evaluation = this.getTaskEvaluation();
    if (!observation || !evaluation) return {};
    return {
      contact_count: Number(observation.contactCount || 0),
      flask_grasp_seen: evaluation.flask.graspSeen,
      flask_support_while_held_seen: evaluation.flask.supportWhileHeldSeen,
      flask_support_contact: evaluation.flask.currentSupportContact,
      beaker_grasp_seen: evaluation.beaker.graspSeen,
      beaker_support_while_held_seen: evaluation.beaker.supportWhileHeldSeen,
      beaker_support_contact: evaluation.beaker.currentSupportContact,
      order_violation: evaluation.orderViolation,
      task_success: evaluation.success,
    };
  }
  getState() {
    return this.lastObservation ? { observation: structuredClone(this.lastObservation), evaluation: this.getTaskEvaluation(), authority: this.getPhysicalAuthorityToken() } : null;
  }
  async applyAction() {
    throw new Error('Legacy OpenArm .pos/source-plant replay is disabled in the physical workspace. Use robobuddy.sim.v1 radians or the bounded OpenArm physical WebMCP schema.');
  }
  async applyPhysicalTargets(targetsRad, { maxSteps = 1000, advanceSeconds = 0 } = {}) {
    this.#assertReady();
    const cleanTargets = finiteTargetMap(targetsRad);
    if (!Number.isInteger(maxSteps) || maxSteps < 1 || maxSteps > 5000) throw new RangeError('maxSteps must be an integer from 1 to 5000');
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
    try {
      const diagnostics = await this.session.getDiagnostics();
      if (!diagnostics?.loaded) return false;
      await this.session.cancelRun('human-stop');
      return true;
    } catch { return false; }
  }
  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.ready = false;
    void this.#disposeSession();
    if (this.canonicalRig) {
      this.robotRoot.remove(this.canonicalRig.root);
      this.canonicalRig.dispose();
      this.canonicalRig = null;
    }
    this.controls?.dispose?.();
    this.renderer?.dispose?.();
  }

  async #ensureCanonicalPresentation() {
    if (this.canonicalRig) return this.canonicalRig;
    const rig = await CanonicalRobotRig.load('openarm');
    const hidden = [];
    rig.root.traverse((node) => {
      if (node.isMesh && NONPHYSICAL_CANONICAL_PARTS.has(node.name)) {
        node.visible = false;
        hidden.push(node.name);
      }
    });
    rig.root.position.fromArray(CANONICAL_OPENARM_MOUNT_TRANSLATION_MM);
    rig.root.userData.presentationAuthority = 'observed MuJoCo joints only';
    rig.root.userData.legacyBaseYawPhysical = false;
    this.robotRoot.add(rig.root);
    this.canonicalRig = rig;
    this.hiddenCanonicalParts = hidden.sort();
    return rig;
  }
  async #createSession() {
    const session = new PhysicsSession(
      new BrowserMuJoCoBackend({ workerUrl: new URL('./openarm-mujoco-worker.js', import.meta.url) }),
      { sessionId: `ide-openarm-v2-${++this.sessionSequence}`, observationBatchSteps: OPENARM_OBSERVATION_BATCH_STEPS },
    );
    this.session = session;
    this.unsubscribeSession = session.subscribe(({ observation }) => this.#consumeObservation(observation));
    await session.loadScene(structuredClone(OPENARM_V2_PHASE5A_SCENE));
  }
  async #disposeSession() {
    this.unsubscribeSession?.();
    this.unsubscribeSession = null;
    if (this.session) this.session.dispose();
    this.session = null;
    this.lastObservation = null;
    this.presentationDirty = false;
  }
  #consumeObservation(observation) {
    if (!observation || this.disposed) return;
    this.lastObservation = structuredClone(observation);
    this.evaluator?.observe(observation);
    // The evaluator consumes every authoritative sample; the scene graph only ever shows the
    // latest one, so presentation is pulled by the render loop instead of pushed per sample.
    // Rendering still never advances physics.
    this.presentationDirty = true;
    this.canvas.dataset.simulationClockS = String(Number(observation.simulationTimeSeconds || 0));
    const evaluation = this.getTaskEvaluation();
    this.canvas.dataset.physicalTaskSuccess = String(Boolean(evaluation?.success));
    this.canvas.dataset.physicalTaskContact = String(Boolean(evaluation?.flask?.graspSeen || evaluation?.beaker?.graspSeen));
    this.canvas.dataset.physicalTaskLift = String(Boolean(evaluation?.flask?.liftSeen || evaluation?.beaker?.liftSeen));
    this.canvas.dataset.physicalTaskCarry = String(Boolean(evaluation?.flask?.carrySeen || evaluation?.beaker?.carrySeen));
    this.canvas.dataset.physicalTaskRelease = String(Boolean(evaluation?.flask?.releaseSeen || evaluation?.beaker?.releaseSeen));
    this.canvas.dataset.physicalTaskSettled = String(Boolean(evaluation?.flask?.settled && evaluation?.beaker?.settled));
  }
  #applyObservation(observation) {
    if (this.canonicalRig) {
      this.canonicalRig.applyPhysicalState(canonicalStateFromObservation(observation));
      this.canonicalRig.root.position.fromArray(CANONICAL_OPENARM_MOUNT_TRANSLATION_MM);
      this.canonicalRig.root.updateMatrixWorld(true);
    }
    for (const [objectId, mesh] of this.objectMeshes) {
      const body = observation.bodies?.[objectId];
      if (!body?.positionM) continue;
      mesh.position.copy(toThreePosition(body.positionM));
      if (body.quaternionWxyz) mesh.quaternion.copy(toThreeQuaternion(body.quaternionWxyz));
    }
  }

  #buildPresentation() {
    const supportMaterial = new THREE.MeshStandardMaterial({ color: PRESENTATION_GROUND_COLOR, roughness: 0.76, metalness: 0.03 });
    const darkMaterial = new THREE.MeshStandardMaterial({ color: 0x30343a, roughness: 0.62, metalness: 0.12 });
    const table = box(0.82, 0.01, 1.10, supportMaterial);
    table.position.copy(toThreePosition([0.41, 0, 1.0]));
    table.name = 'visual-cell-table';
    this.workcellRoot.add(table);

    const mount = box(0.08, 0.08, 0.12, darkMaterial);
    mount.position.copy(toThreePosition([0.185, 0, 1.31]));
    mount.name = 'visual-openarm-source-mount';
    this.workcellRoot.add(mount);

    const fixtures = [
      ['left-source', box(0.09, 0.03, 0.09, supportMaterial), [0.509, 0.1535, 1.020]],
      ['left-hotplate', box(0.116, 0.03, 0.108, darkMaterial), [0.608, 0.1535, 1.020]],
      ['right-source', box(0.07, 0.07, 0.07, supportMaterial), [0.509, -0.1535, 1.040]],
    ];
    for (const [name, mesh, position] of fixtures) { mesh.name = `visual-${name}`; mesh.position.copy(toThreePosition(position)); this.workcellRoot.add(mesh); }
    const post = cylinder(0.006, 0.07, darkMaterial); post.position.copy(toThreePosition([0.608, -0.235, 1.040])); post.name = 'visual-ring-post'; this.workcellRoot.add(post);
    const gauze = cylinder(0.048, 0.006, supportMaterial); gauze.position.copy(toThreePosition([0.608, -0.1535, 1.072])); gauze.name = 'visual-ring-gauze'; this.workcellRoot.add(gauze);

    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.40, depthWrite: false });
    for (const [center, size] of [[[0.608, 0.1535, 1.036], [0.034, 0.001, 0.026]], [[0.608, -0.1535, 1.076], [0.042, 0.001, 0.042]]]) {
      const marker = box(size[0], size[1], size[2], markerMaterial); marker.position.copy(toThreePosition(center)); marker.userData.presentationOnly = true; this.targetMarkers.push(marker); this.workcellRoot.add(marker);
    }

    const flaskGroup = new THREE.Group();
    const flaskMaterial = new THREE.MeshStandardMaterial({ color: 0x69b8d8, transparent: true, opacity: 0.80, roughness: 0.28 });
    const flaskBody = cylinder(0.039, 0.055, flaskMaterial); flaskBody.position.y = -29.5;
    const flaskShoulder = cylinder(0.031, 0.028, flaskMaterial); flaskShoulder.position.y = 12;
    const flaskNeck = cylinder(0.015, 0.028, flaskMaterial); flaskNeck.position.y = 40;
    flaskGroup.add(flaskBody, flaskShoulder, flaskNeck); this.objectMeshes.set('flask', flaskGroup); this.workcellRoot.add(flaskGroup);
    const beakerMaterial = new THREE.MeshStandardMaterial({ color: 0x8dc8e3, transparent: true, opacity: 0.76, roughness: 0.30 });
    const beaker = cylinder(0.025, 0.060, beakerMaterial); this.objectMeshes.set('beaker', beaker); this.workcellRoot.add(beaker);
    this.setHighContrastScene(this.highContrast);
  }
  #assertReady() { this.#assertNotDisposed(); if (!this.isReady()) throw new Error('OpenArm physical session is not ready'); }
  #assertNotDisposed() { if (this.disposed) throw new Error('OpenArm physical simulator is disposed'); }
}
