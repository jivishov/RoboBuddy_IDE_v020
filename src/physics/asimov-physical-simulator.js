import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/loaders/STLLoader.js';
import { BrowserMuJoCoBackend } from './browser-mujoco-backend.js';
import { PhysicsSession } from './session.js';
import { asimovSceneById } from './asimov-scene.js';
import { ASIMOV_SOURCE } from './asimov-generated.js';
import { ASIMOV_JOINT_ORDER, ASIMOV_PHYSICS_TIMESTEP_SECONDS } from './asimov-controller.js';

const ROOT = new URL('../../',import.meta.url);
async function checkedBytes(asset, hash, signal) {
  const url=new URL(asset,ROOT);
  if(url.origin!==ROOT.origin || !url.pathname.startsWith(new URL('models/asimov/',ROOT).pathname)) throw new Error('Invalid Asimov asset path');
  const response=await fetch(url,{signal});
  if(!response.ok) throw new Error(`Asimov asset ${asset}: HTTP ${response.status}`);
  const bytes=await response.arrayBuffer();
  const digest=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),v=>v.toString(16).padStart(2,'0')).join('');
  if(digest!==hash) throw new Error(`Asimov asset integrity mismatch: ${asset}`);
  return bytes;
}
function putPose(object, position, quaternion) {
  object.position.fromArray(position);
  const [w,x,y,z]=quaternion; object.quaternion.set(x,y,z,w);
}

/** Source STL instances consume observed world-body poses; rendering never steps physics. */
export class AsimovPhysicalSimulator {
  constructor(canvas) {
    this.canvas=canvas; this.disposed=false; this.ready=false; this.session=null; this.lastObservation=null;
    this.renderedFrames=0; this.highContrast=true; this.sequence=0; this.groups=new Map(); this.geometries=new Map();
    this.abort=new AbortController();
    this.scene=new THREE.Scene(); this.scene.background=new THREE.Color(0xb4bcc0);
    this.camera=new THREE.PerspectiveCamera(45,1,.01,30); this.camera.position.set(1.8,1.35,2.1);
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true}); this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
    this.controls=new OrbitControls(this.camera,canvas); this.controls.enableDamping=true;
    this.scene.add(new THREE.HemisphereLight(0xffffff,0x30404a,1.8));
    const light=new THREE.DirectionalLight(0xffffff,2.2); light.position.set(1.2,2.4,1.2); this.scene.add(light);
    // Whole physical world uses SI metres/Z-up. One presentation-only basis converts to Three Y-up.
    this.world=new THREE.Group(); this.world.rotation.x=-Math.PI/2; this.scene.add(this.world);
    this.material=new THREE.MeshStandardMaterial({color:0xbac0c3,roughness:.65,metalness:.2});
    this.floorMaterial=new THREE.MeshStandardMaterial({color:0x687378,roughness:.9,side:THREE.DoubleSide});
    const floor=new THREE.Mesh(new THREE.PlaneGeometry(4,4),this.floorMaterial); this.world.add(floor);
    const grid=new THREE.GridHelper(4,40,0x334155,0x4b5563); grid.position.y=.001; this.scene.add(grid);
    this.mount=new THREE.Mesh(new THREE.BoxGeometry(.05,.12,1),new THREE.MeshStandardMaterial({color:0x34516b}));
    this.mount.position.set(-.17,0,.5); this.mount.visible=false; this.world.add(this.mount);
    this.fit(); this.resize();
  }
  async setScenario(profileId,scenario) {
    if(profileId!=='asimov'||scenario?.simulationMode!=='physical_mujoco') throw new Error('Asimov requires a declared physical scene; no kinematic fallback exists');
    const scene=asimovSceneById(scenario.physicalSceneId);
    if(this.disposed) throw new Error('Disposed Asimov simulator');
    const generation=++this.sequence; this.ready=false;
    this.unsubscribe?.(); this.session?.dispose();
    this.selectedScene=scene; this.mount.visible=scene.id==='asimov-mounted';
    const session=new PhysicsSession(new BrowserMuJoCoBackend({workerUrl:new URL('./asimov-mujoco-worker.js',import.meta.url),setupOperations:['set_actuation']}),{observationBatchSteps:4});
    this.session=session; this.unsubscribe=session.subscribe(({observation})=>{if(!this.disposed && generation===this.sequence) this.consume(observation);});
    try {
      await Promise.all([this.loadMeshes(),session.loadScene(structuredClone(scene))]);
      if(this.disposed || generation!==this.sequence) throw new Error('Superseded Asimov scene');
      this.ready=true; Object.assign(this.canvas.dataset,{simulatorBackend:'browser-mujoco',simulationAuthority:'physics-session',physicalSceneId:scene.id,
        physicalSceneRevision:scene.revision,modelPackageId:scene.modelPackage,asimovRootMode:this.lastObservation.root.mode,asimovWalking:'unsupported'});
      this.applyObservation(); this.fit(); return true;
    } catch(error) {session.dispose(); this.ready=false; throw error;}
  }
  async loadMeshes() {
    if(this.groups.size) return;
    const manifest=JSON.parse(new TextDecoder().decode(await checkedBytes('models/asimov/visual/manifest.json',ASIMOV_SOURCE.visualManifestSha256,this.abort.signal)));
    const loader=new STLLoader();
    // Sequential decode bounds transient RAM for the full-resolution mesh; shared feet decode once.
    for(const [file,pin] of Object.entries(manifest.meshes)) {
      const compressed=await checkedBytes(pin.asset,pin.sha256,this.abort.signal);
      if(compressed.byteLength!==pin.bytes) throw new Error('Asimov compressed size mismatch');
      const bytes=await new Response(new Blob([compressed]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
      if(bytes.byteLength!==pin.rawBytes) throw new Error('Asimov raw mesh size mismatch');
      const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),v=>v.toString(16).padStart(2,'0')).join('');
      if(hash!==pin.rawSha256) throw new Error('Asimov raw mesh integrity mismatch');
      if(this.disposed) throw new Error('Disposed Asimov asset load');
      this.geometries.set(file,loader.parse(bytes));
    }
    for(const instance of manifest.instances) {
      let group=this.groups.get(instance.body);
      if(!group) {group=new THREE.Group(); group.name=instance.body; this.groups.set(instance.body,group); this.world.add(group);}
      const mesh=new THREE.Mesh(this.geometries.get(instance.file),this.material);
      putPose(mesh,instance.positionM,instance.quaternionWxyz); mesh.scale.fromArray(instance.scale); group.add(mesh);
    }
  }
  consume(observation) {
    this.lastObservation=structuredClone(observation);
    this.canvas.dataset.simulationClockS=String(observation.simulationTimeSeconds);
    this.canvas.dataset.asimovPelvisZM=String(observation.root?.positionM?.[2]);
    this.canvas.dataset.asimovContactCount=String(observation.contactCount);
  }
  applyObservation() {
    for(const [name,group] of this.groups) {
      const body=this.lastObservation?.bodies?.[name];
      if(body) putPose(group,body.positionM,body.quaternionWxyz);
    }
  }
  renderFrame() {
    if(this.disposed) return;
    this.applyObservation(); this.controls.update(); this.renderer.render(this.scene,this.camera);
    this.canvas.dataset.renderedFrames=String(++this.renderedFrames);
  }
  resize() {
    if(this.disposed) return;
    const w=Math.max(1,this.canvas.clientWidth||640), h=Math.max(1,this.canvas.clientHeight||480);
    this.renderer.setSize(w,h,false); this.camera.aspect=w/h; this.camera.updateProjectionMatrix();
  }
  fit() { this.camera.position.set(1.8,1.35,2.1); this.controls.target.set(0,this.mount?.visible?1:.65,0); this.controls.update(); }
  setHighContrastScene(v) {this.highContrast=Boolean(v); return this.highContrast;}
  isHighContrastSceneEnabled() {return this.highContrast;}
  isReady() {return !!(!this.disposed&&this.ready&&this.session?.robotId&&this.lastObservation);}
  assertReady() {if(!this.isReady()) throw new Error('Asimov physical session is not ready');}
  getPhysicalSession() {return this.session;}
  getPhysicalAuthorityToken() {return this.isReady()?{sessionId:this.session.sessionId,epoch:this.session.epoch,sceneRevision:this.session.sceneRevision,robotId:this.session.robotId}:null;}
  async applyPhysicalTargets(targetsRad,{advanceSeconds=0,maxSteps=4000}={}) {
    this.assertReady(); const result=await this.session.sendCommand({type:'set_joint_targets',targetsRad},{maxSteps});
    if(advanceSeconds>0) await this.advanceTime(advanceSeconds); return result;
  }
  applyAction(targetsRad,options) {return this.applyPhysicalTargets(targetsRad,options);}
  async applyLowLevelCommands(commands,{advanceSeconds=0,maxSteps=4000}={}) {
    this.assertReady(); const result=await this.session.sendCommand({type:'set_lowlevel_targets',commands},{maxSteps});
    if(advanceSeconds>0) await this.advanceTime(advanceSeconds); return result;
  }
  async advanceTime(seconds,{maxSeconds=20}={}) {
    this.assertReady(); if(typeof seconds!=='number'||!Number.isFinite(seconds)||seconds<=0||seconds>maxSeconds) throw new RangeError('Asimov advance outside bounded simulation-time interval');
    const steps=Math.round(seconds/ASIMOV_PHYSICS_TIMESTEP_SECONDS); if(steps<1) throw new RangeError('Advance needs at least one physics step');
    return this.session.advanceSteps(steps);
  }
  async reset() {this.assertReady(); await this.session.reset({reason:'explicit-user-reset'}); return true;}
  pause() {this.assertReady(); return this.session.pause();}
  resume() {this.assertReady(); return this.session.resume();}
  async stop() {
    this.assertReady(); return this.applyPhysicalTargets(Object.fromEntries(ASIMOV_JOINT_ORDER.map(id=>[id,this.lastObservation.joints[id].positionRad])),{maxSteps:1});
  }
  setActuationEnabled(enabled) {this.assertReady(); return this.session.applySetup({type:'set_actuation',enabled});}
  getState() {
    const o=this.lastObservation; if(!o) return null;
    return {simulation_time_s:o.simulationTimeSeconds,root:{mode:o.root.mode,position_m:o.root.positionM,quaternion_wxyz:o.root.quaternionWxyz,tilt_rad:o.root.tiltRad,
      linear_velocity_m_s:o.root.linearVelocityMS,angular_velocity_rad_s:o.root.angularVelocityRadS},
      joints:Object.fromEntries(Object.entries(o.joints).map(([id,j])=>[id,{position_rad:j.positionRad,velocity_rad_s:j.velocityRadS,effort_nm:j.effortNm,
        requested_target_rad:j.requestedTargetRad,accepted_target_rad:j.acceptedTargetRad,command_bounded:j.commandBounded,effort_limit_nm:j.effortLimitNm}])),
      contacts:o.contacts,controller_mode:o.controller.id,actuation_enabled:o.actuationEnabled,walking:'unsupported'};
  }
  getContacts() {return {count:this.lastObservation?.contactCount??0,readable:!!this.lastObservation?.contactsReadable,...this.lastObservation?.contactClasses};}
  getTelemetry() {return {backend:'browser-mujoco',authority:'physics-session',simulationTimeSeconds:this.lastObservation?.simulationTimeSeconds??0,
    rootMode:this.lastObservation?.root?.mode,totalMassKg:ASIMOV_SOURCE.totalMassKg,controllerId:this.lastObservation?.controller?.id,walking:'unsupported'};}
  getTaskEvaluation() {return {status:'observation-only',standing:null,walking:'unsupported',syntheticSuccessEvents:false};}
  getPresentationAudit() {return {sourceRevision:ASIMOV_SOURCE.revision,jointCount:23,meshCount:this.geometries.size,bodyCount:this.groups.size,drivesFromObservation:!!this.lastObservation};}
  getPresentationAlignment() {
    this.applyObservation(); let maximum=0;
    for(const [id,g] of this.groups) maximum=Math.max(maximum,g.position.distanceTo(new THREE.Vector3(...this.lastObservation.bodies[id].positionM)));
    return {comparedBodies:this.groups.size,maxBodyErrorMm:maximum*1000};
  }
  dispose() {
    if(this.disposed) return; this.disposed=true; this.ready=false; this.sequence++; this.abort.abort();
    this.unsubscribe?.(); this.session?.dispose(); this.session=null;
    for(const geometry of this.geometries.values()) geometry.dispose(); this.geometries.clear(); this.groups.clear();
    this.scene.traverse(o=>{o.geometry?.dispose?.(); if(o.material) (Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose?.());});
    this.controls.dispose(); this.renderer.dispose();
    for(const key of ['asimovRootMode','asimovWalking','asimovPelvisZM','asimovContactCount','simulationAuthority','physicalSceneId','physicalSceneRevision','modelPackageId','renderedFrames']) delete this.canvas.dataset[key];
  }
}
