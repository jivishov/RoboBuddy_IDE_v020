import { object, identifier, number, transform, rotate, inverseRotate, multiplyQuaternion, sub, dot, bounds, IDENTITY } from './openarm-general-geometry.js';
import { graspState, contactsForObject, contactEvidenceComplete, usableContact, activeOrUncertainContact } from './openarm-observation.js';
export const GENERAL_TASK_VERSION='robobuddy.lab.task.v2';
const time=o=>o.simulationTimeSeconds??o.simulationTime;
const record=(o,id)=>o.openarm?.equipment?.find(e=>e.bodyId===id);
const validBody=b=>b&&Array.isArray(b.positionM)&&b.positionM.length===3&&b.positionM.every(Number.isFinite)&&Array.isArray(b.quaternionWxyz)&&b.quaternionWxyz.length===4&&b.quaternionWxyz.every(Number.isFinite)&&Math.abs(Math.hypot(...b.quaternionWxyz)-1)<.001;
const stationary=(b,dynamic)=>!dynamic||Array.isArray(b.linearVelocityMS)&&b.linearVelocityMS.length===3&&Array.isArray(b.angularVelocityRadS)&&b.angularVelocityRadS.length===3&&b.linearVelocityMS.every(Number.isFinite)&&b.angularVelocityRadS.every(Number.isFinite)&&Math.hypot(...b.linearVelocityMS)<.01&&Math.hypot(...b.angularVelocityRadS)<.15;
export function worldPort(body,port){return {...port,positionM:transform(port.positionM,body.positionM,body.quaternionWxyz),quaternionWxyz:multiplyQuaternion(body.quaternionWxyz,port.quaternionWxyz??IDENTITY)};}
function corners(b){const p=[];for(const x of [b.min[0],b.max[0]])for(const y of [b.min[1],b.max[1]])for(const z of [b.min[2],b.max[2]])p.push([x,y,z]);return p;}
export function validateGeneralTask(raw,o){
  object(raw,['schema_version','id','type','object_id','receiver_id','target_port','object_port','side','insertion_depth_m','acknowledge_simulation_only'],'TaskSpec');
  if(raw.schema_version!==GENERAL_TASK_VERSION)throw new TypeError(`TaskSpec requires ${GENERAL_TASK_VERSION}`);
  const type=raw.type;if(!['dry_transfer','insert'].includes(type))throw new TypeError('Supported tasks: dry_transfer onto an authored support, insert a cylindrical peg into an authored open bore');
  if(raw.acknowledge_simulation_only!==true)throw new TypeError('Task definition requires acknowledge_simulation_only:true; it does not assert real-world validity');
  const task={schema_version:GENERAL_TASK_VERSION,id:identifier(raw.id),type,object_id:`lab_${identifier(raw.object_id)}`,receiver_id:`lab_${identifier(raw.receiver_id)}`,target_port:identifier(raw.target_port),side:raw.side};
  if(!['left','right'].includes(task.side))throw new TypeError('Task side must be left or right');
  const a=record(o,task.object_id),b=record(o,task.receiver_id);
  if(!a?.dynamic||!b||a===b)throw new TypeError('Task requires a distinct dynamic object and a declared receiver');
  const target=b.ports?.find(p=>p.id===task.target_port);
  if(!target||target.type!==(type==='dry_transfer'?'support':'opening'))throw new TypeError('Target port has the wrong or missing geometry-derived feature');
  const warnings=[];
  let nominalClearanceM=null,uncertaintyAllowanceM=null;
  if(type==='insert'){
    task.object_port=identifier(raw.object_port);
    const peg=a.ports?.find(p=>p.id===task.object_port);
    if(peg?.type!=='peg')throw new TypeError('Insertion requires a geometry-derived cylindrical peg port');
    nominalClearanceM=target.radiusM-peg.radiusM;
    if(nominalClearanceM<=0)throw new RangeError('Peg does not fit the conservative polygonal bore; geometry must be corrected, not the success tolerance');
    const maxDepth=Math.min(peg.lengthM,target.depthM);
    task.insertion_depth_m=number(raw.insertion_depth_m??maxDepth*.7,.005,maxDepth,'insertion_depth_m');
    const u=[a,b].map(e=>e.quantityEvidence?.geometry?.uncertainty_m);
    uncertaintyAllowanceM=u.every(Number.isFinite)?u[0]+u[1]:null;
    if(uncertaintyAllowanceM===null||uncertaintyAllowanceM>=nominalClearanceM)warnings.push('Real fit is unresolved: declared dimensional uncertainty is unknown or exceeds nominal radial clearance. Obtain measurements before a real-world fit claim.');
  }else if(raw.object_port!==undefined||raw.insertion_depth_m!==undefined)throw new TypeError('dry_transfer does not accept insertion fields');
  if([a,b].some(e=>e.quantityEvidence?.geometry?.source!=='user_measured'))warnings.push('Task geometry has not been independently measured by this application.');
  warnings.push('Preflight checks feature definitions and nominal dimensions, not reachability, other-part bore obstruction, graspability, load capacity, or a collision-free path.');
  return {task,preflight:{status:'nominal_geometry_checked',nominalClearanceM,uncertaintyAllowanceM,realWorldFitEstablished:false,collisionFreePathVerified:false,reachabilityVerified:false,hardwareValidated:false,warnings}};
}
/** Instantaneous outcome only. History is evaluated separately and cannot be supplied by a tool caller. */
export function generalPlacementState(o,task){
  const no=reason=>({placed:false,reason});
  const a=record(o,task.object_id),b=record(o,task.receiver_id),ab=o.bodies?.[task.object_id],bb=o.bodies?.[task.receiver_id];
  if(!a||!b||!validBody(ab)||!validBody(bb)||!contactEvidenceComplete(o))return no('Missing complete authoritative body/contact state');
  const port=b.ports?.find(p=>p.id===task.target_port);if(!port)return no('Target feature unavailable');
  const target=worldPort(bb,port),up=rotate(target.quaternionWxyz,[0,0,1]);
  const geometryIds=new Set(a.geometryIds),targetIds=new Set(port.geometryIds);
  const contacts=contactsForObject(o,task.object_id),other=c=>geometryIds.has(c.geom1Name)?c.geom2Name:c.geom1Name;
  const receiving=contacts.filter(c=>targetIds.has(other(c))&&Number.isFinite(c.normalForceN)&&c.normalForceN>=0&&Number.isFinite(c.distanceM)&&c.distanceM>=-.002);
  const receiverForceN=receiving.reduce((s,c)=>s+c.normalForceN,0);
  const released=['left','right'].every(side=>!graspState(o,side,task.object_id).anyGripperContact);
  const otherSupport=contacts.some(c=>!targetIds.has(other(c))&&activeOrUncertainContact(c));
  const excessivePenetration=contacts.some(c=>!Number.isFinite(c.distanceM)||c.distanceM<-.002);
  const settled=stationary(ab,a.dynamic)&&stationary(bb,b.dynamic);
  let geometryFits=false,details={};
  if(task.type==='insert'){
    const p=a.ports?.find(p=>p.id===task.object_port);if(p?.type!=='peg')return no('Peg unavailable');
    const peg=worldPort(ab,p),axis=rotate(peg.quaternionWxyz,[0,0,1]),r=sub(peg.positionM,target.positionM),depth=-dot(r,up);
    const radial=Math.hypot(...r.map((v,i)=>v+depth*up[i])),alignment=Math.max(-1,Math.min(1,dot(axis,up))),tilt=Math.acos(alignment);
    const clearance=port.radiusM-p.radiusM;
    geometryFits=up[2]>.98&&alignment>0&&depth>=task.insertion_depth_m&&depth<=port.depthM+.001&&radial+Math.sin(tilt)*Math.min(depth,p.lengthM)<=clearance+.00025;
    details={insertionDepthM:depth,radialOffsetM:radial,tiltRad:tilt,radialClearanceM:clearance};
  }else{
    const points=corners(a.localBounds).map(p=>inverseRotate(target.quaternionWxyz,sub(transform(p,ab.positionM,ab.quaternionWxyz),target.positionM)));
    const envelope=bounds(points);
    const within=port.halfExtentsM?points.every(p=>Math.abs(p[0])<=port.halfExtentsM[0]+.001&&Math.abs(p[1])<=port.halfExtentsM[1]+.001):points.every(p=>Math.hypot(p[0],p[1])<=port.radiusM+.001);
    geometryFits=up[2]>.98&&within&&Math.abs(envelope.min[2])<.003;
    details={relativeEnvelope:envelope};
  }
  const supported=receiverForceN>Math.max(.01,a.mass_kg*9.81*.2);
  return {placed:geometryFits&&released&&settled&&supported&&!otherSupport&&!excessivePenetration,geometryFits,released,settled,supported,otherSupport,excessivePenetration,receiverForceN,...details,evidence:'instantaneous MuJoCo ground truth, not reconstruction accuracy or transfer history'};
}
export class GeneralTaskEvaluator{
  constructor(validated,o,revision){
    this.task=structuredClone(validated.task);this.preflight=structuredClone(validated.preflight);this.revision=revision;
    this.initial=structuredClone(o.bodies?.[this.task.object_id]);
    if(!validBody(this.initial)||['left','right'].some(s=>graspState(o,s,this.task.object_id).anyGripperContact))throw new Error('Define the task before the robot grasps the object');
    if(!contactEvidenceComplete(o))throw new Error('Task definition requires complete physical observations');
    const initialPlacement=generalPlacementState(o,this.task);
    if(initialPlacement.placed||initialPlacement.geometryFits)throw new Error('The object is already in the receiving geometry; setup must not be credited as a robot transfer');
    this.lastTime=time(o);this.invalidated=null;this.failure=null;this.events=[];this.flags={sourceSupported:false,grasp:false,lift:false,carry:false,release:false,retreat:false};this.sourceDwell=0;this.graspDwell=0;this.settleDwell=0;this.current={placed:false};
  }
  invalidate(reason){this.invalidated=reason;this.settleDwell=0;}
  mark(name,o,extra={}){if(!this.flags[name]){this.flags[name]=true;this.events.push({event:name,timeSeconds:time(o),objectPose:structuredClone(o.bodies[this.task.object_id]),...extra});}}
  observe(o){
    if(this.invalidated)return;
    const t=time(o),dt=t-this.lastTime;if(dt===0)return;
    if(!Number.isFinite(dt)||dt<0||dt>.051){this.invalidate('Non-monotonic or insufficiently sampled observations; restart an explicit trial');return;}this.lastTime=t;
    const a=record(o,this.task.object_id),body=o.bodies?.[this.task.object_id];
    if(!a||!validBody(body)||!contactEvidenceComplete(o)){this.invalidate('Incomplete physical observations');return;}
    const grip=graspState(o,this.task.side,this.task.object_id),contacts=contactsForObject(o,this.task.object_id);
    if(contacts.some(c=>!Number.isFinite(c.distanceM)||c.distanceM<-.002)){this.failure={timeSeconds:t,objectPose:structuredClone(body),contacts:structuredClone(contacts.filter(c=>!Number.isFinite(c.distanceM)||c.distanceM<-.002).slice(0,8))};this.invalidate('Excessive or unreadable object penetration');return;}
    const own=new Set(a.geometryIds),other=c=>own.has(c.geom1Name)?c.geom2Name:c.geom1Name;
    const nonGripper=contacts.filter(c=>!/^finger_|^ee_base_link_/.test(other(c))&&usableContact(c));
    const released=['left','right'].every(s=>!graspState(o,s,this.task.object_id).anyGripperContact);
    if(!this.flags.grasp){this.sourceDwell=released&&nonGripper.length?this.sourceDwell+dt:0;if(this.sourceDwell>=.05)this.mark('sourceSupported',o);}
    this.graspDwell=grip.bilateralContact&&!grip.anyPalmContact?this.graspDwell+dt:0;
    if(this.flags.sourceSupported&&this.graspDwell>=.06)this.mark('grasp',o,{contact:grip});
    if(this.flags.grasp&&grip.bilateralContact&&!nonGripper.length&&body.positionM[2]-this.initial.positionM[2]>=.02&&!this.flags.lift){this.mark('lift',o);this.liftPosition=[...body.positionM];}
    if(this.flags.lift&&grip.bilateralContact&&Math.hypot(...sub(body.positionM,this.liftPosition))>=.02)this.mark('carry',o);
    this.current=generalPlacementState(o,this.task);
    const firstRelease=this.flags.carry&&released&&!this.flags.release;
    if(firstRelease){this.mark('release',o);this.releasePinch=structuredClone(o.openarm?.pinchReferences?.[this.task.side]?.positionM??null);}
    const pinch=o.openarm?.pinchReferences?.[this.task.side]?.positionM;
    if(this.flags.release&&released&&pinch?.length===3&&this.releasePinch?.length===3&&Math.hypot(...sub(pinch,this.releasePinch))>=.04&&Math.hypot(...sub(pinch,body.positionM))>=.08)this.mark('retreat',o);
    this.settleDwell=this.flags.retreat&&this.current.placed?this.settleDwell+dt:0;
  }
  snapshot(){return {schemaVersion:GENERAL_TASK_VERSION,task:structuredClone(this.task),sceneRevision:this.revision,preflight:structuredClone(this.preflight),success:!this.invalidated&&this.settleDwell>=.2,status:this.invalidated?'invalidated':this.settleDwell>=.2?'completed':'in_progress',invalidated:this.invalidated,failure:structuredClone(this.failure),observedSequence:{...this.flags},settledDwellSeconds:this.settleDwell,currentPlacement:structuredClone(this.current),events:structuredClone(this.events),hardwareValidated:false,imageReconstructionVerified:false,scope:'observed contact-driven simulated transfer in this frozen scene; not calibration or a general motion-planning guarantee'};}
}
