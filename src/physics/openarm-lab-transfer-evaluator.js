import { contactsForObject, graspState, usableContact, worldPoint } from './openarm-observation.js';

const clone = value => structuredClone(value);
const PHASES = Object.freeze(['grasp','lift','transport','receivingRegion','support','release','settled','retreat']);
const speed = values => Array.isArray(values) && values.every(Number.isFinite) ? Math.hypot(...values) : Infinity;
const clamp = value => Math.max(-1, Math.min(1, value));
const quatAngle = (a, b) => {
  if (![a,b].every(q => Array.isArray(q) && q.length === 4 && q.every(Number.isFinite))) return Infinity;
  return 2 * Math.acos(clamp(Math.abs(a.reduce((sum, value, index) => sum + value * b[index], 0))));
};
const zAxis = q => {
  if (!Array.isArray(q) || q.length !== 4 || q.some(value => !Number.isFinite(value))) return null;
  const [w,x,y,z] = q;
  return [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)];
};
const axisAngle = (a,b) => {
  const u=zAxis(a),v=zAxis(b); if(!u||!v)return Infinity;
  return Math.acos(clamp(u.reduce((sum,value,index)=>sum+value*v[index],0)));
};
function localPoint(body, world) {
  const [w,x,y,z]=body.quaternionWxyz;
  const p=world.map((value,index)=>value-body.positionM[index]);
  const t=[-2*(y*p[2]-z*p[1]),-2*(z*p[0]-x*p[2]),-2*(x*p[1]-y*p[0])];
  return [p[0]+w*t[0]-y*t[2]+z*t[1],p[1]+w*t[1]-z*t[0]+x*t[2],p[2]+w*t[2]-x*t[1]+y*t[0]];
}
function geometryPairMatches(contact, rule) {
  const pair=[contact.geom1Name,contact.geom2Name], [a,b]=String(rule).split('|');
  if(!a||!b)return false;
  const match=(name,pattern)=>pattern.endsWith('*')?String(name||'').startsWith(pattern.slice(0,-1)):name===pattern;
  return (match(pair[0],a)&&match(pair[1],b))||(match(pair[0],b)&&match(pair[1],a));
}
const contactIsPhysical = contact => Number.isFinite(contact?.distanceM) && Number.isFinite(contact?.normalForceN)
  && (contact.distanceM <= 0 || contact.normalForceN > .01);

export class OpenArmLabTransferEvaluator {
  constructor(sceneSpec=null,taskSpec=null){this.configure(sceneSpec,taskSpec);}
  configure(sceneSpec,taskSpec){this.sceneSpec=sceneSpec?clone(sceneSpec):null;this.task=taskSpec?clone(taskSpec):null;this.reset();}
  reset(){
    this.initialized=false;this.initialObjectPositionM=null;this.initialAlreadySatisfied=false;this.firstSimulationTimeS=null;
    this.graspStartS=null;this.settleStartS=null;this.events=[];this.last=null;this.success=false;this.failure=null;
    this.flags={grasp:false,lift:false,transport:false,receivingRegion:false,support:false,release:false,settled:false,retreat:false,excessivePenetration:false,prohibitedContact:false};
  }
  #event(type,observation,detail={}){
    if(PHASES.includes(type)&&this.events.some(event=>event.type===type))return;
    this.events.push({type,simulation_time_s:observation.simulationTimeSeconds,...detail});
  }
  #records(observation){
    if(!this.task)return null;
    const object=(observation.openarm?.equipment||[]).find(record=>record.id===this.task.object_id);
    const receiver=(observation.openarm?.equipment||[]).find(record=>record.id===this.task.receiver_id);
    if(!object||!receiver)return null;
    return {object,receiver,objectBody:observation.bodies?.[object.bodyId],receiverBody:observation.bodies?.[receiver.bodyId]};
  }
  #placement(observation,records){
    const {object,receiver,objectBody,receiverBody}=records;
    if(!objectBody||!receiverBody)return{inside:false,supported:false,orientationOk:false};
    const local=localPoint(receiverBody,objectBody.positionM);
    const half=receiver.affordances?.receivingHalfExtentsM||[receiver.dimensions_m[0]/2,receiver.dimensions_m[1]/2];
    const bottom=Number(receiver.affordances?.receivingBottomM?.[2]??0), margin=this.task.tolerances.position_m;
    const inside=Math.abs(local[0])<=Math.max(0,half[0]-margin)&&Math.abs(local[1])<=Math.max(0,half[1]-margin)&&Math.abs(local[2]-bottom)<=margin;
    const supportGeom=receiver.affordances?.supportGeometryId||`lab_${receiver.id}_base`;
    const supported=contactsForObject(observation,object.bodyId).some(contact=>usableContact(contact)&&[contact.geom1Name,contact.geom2Name].includes(supportGeom));
    // Cylindrical vials are yaw-symmetric, but tilt and inversion still matter. Blocks use the
    // full quaternion orientation tolerance.
    const orientationErrorRad=object.kind==='vial'?axisAngle(objectBody.quaternionWxyz,receiverBody.quaternionWxyz):quatAngle(objectBody.quaternionWxyz,receiverBody.quaternionWxyz);
    return {inside,supported,orientationOk:orientationErrorRad<=this.task.tolerances.orientation_rad,orientationErrorRad,local,supportGeom};
  }
  observe(observation){
    if(!observation||!this.task||!this.sceneSpec){this.last={observation:observation?clone(observation):null};return this.snapshot();}
    const records=this.#records(observation);
    if(!records?.objectBody||!records?.receiverBody){this.failure={code:'MISSING_TASK_BODY',detail:'The frozen TaskSpec no longer resolves in the active scene.'};this.last={observation:clone(observation)};return this.snapshot();}
    this.last={observation:clone(observation),records};
    const {object,objectBody}=records, grasp=graspState(observation,this.task.side,object.bodyId), placement=this.#placement(observation,records);
    const quiet=speed(objectBody.linearVelocityMS)<=this.task.tolerances.settle_speed_ms&&speed(objectBody.angularVelocityRadS)<=this.task.tolerances.settle_angular_speed_rads;
    if(!this.initialized){
      this.initialized=true;this.firstSimulationTimeS=observation.simulationTimeSeconds;this.initialObjectPositionM=[...objectBody.positionM];
      this.initialAlreadySatisfied=placement.inside&&placement.orientationOk&&placement.supported&&quiet;
      if(this.initialAlreadySatisfied)this.events.push({type:'already_satisfied',simulation_time_s:observation.simulationTimeSeconds,transfer_credit:false});
    }

    // Treat deep penetration involving any authored lab geometry as a task failure signal, not
    // only penetration of the transferred object itself.
    const excessive=(observation.contacts||[]).some(contact=>Number.isFinite(contact.distanceM)&&contact.distanceM<-this.task.tolerances.max_penetration_m
      && [contact.geom1Name,contact.geom2Name].some(name=>typeof name==='string'&&name.startsWith('lab_')));
    if(excessive&&!this.flags.excessivePenetration)this.#event('excessivePenetration',observation);
    this.flags.excessivePenetration ||= excessive;
    const prohibited=(observation.contacts||[]).some(contact=>contactIsPhysical(contact)&&this.task.prohibited_contacts.some(rule=>geometryPairMatches(contact,rule)));
    if(prohibited&&!this.flags.prohibitedContact)this.#event('prohibitedContact',observation);
    this.flags.prohibitedContact ||= prohibited;

    // Ordered application-owned task state machine. A condition observed before its predecessor
    // cannot be banked for later success.
    if(!this.flags.grasp){
      if(grasp.bilateralContact){
        if(this.graspStartS==null)this.graspStartS=observation.simulationTimeSeconds;
        if(observation.simulationTimeSeconds-this.graspStartS>=.06){this.flags.grasp=true;this.#event('grasp',observation,{dwell_s:observation.simulationTimeSeconds-this.graspStartS});}
      }else this.graspStartS=null;
    }
    const lifted=this.flags.grasp&&grasp.bilateralContact&&objectBody.positionM[2]>=this.initialObjectPositionM[2]+.02;
    if(lifted&&!this.flags.lift){this.flags.lift=true;this.#event('lift',observation);}
    const horizontal=Math.hypot(objectBody.positionM[0]-this.initialObjectPositionM[0],objectBody.positionM[1]-this.initialObjectPositionM[1]);
    if(this.flags.lift&&grasp.bilateralContact&&horizontal>=.04&&!this.flags.transport){this.flags.transport=true;this.#event('transport',observation,{horizontal_displacement_m:horizontal});}
    if(this.flags.transport&&placement.inside&&placement.orientationOk&&!this.flags.receivingRegion){this.flags.receivingRegion=true;this.#event('receivingRegion',observation,{orientation_error_rad:placement.orientationErrorRad});}
    if(this.flags.receivingRegion&&placement.supported&&grasp.bilateralContact&&!this.flags.support){this.flags.support=true;this.#event('support',observation,{support_geom:placement.supportGeom});}
    if(this.flags.support&&observation.contactsReadable===true&&!grasp.anyGripperContact&&!this.flags.release){this.flags.release=true;this.#event('release',observation);}

    const sequenceRequired=this.task.required_action_sequence!==false;
    const settleEligible=(sequenceRequired?this.flags.release:true)&&placement.inside&&placement.orientationOk&&placement.supported&&quiet&&!this.flags.excessivePenetration&&!this.flags.prohibitedContact;
    if(settleEligible){
      if(this.settleStartS==null)this.settleStartS=observation.simulationTimeSeconds;
      if(observation.simulationTimeSeconds-this.settleStartS>=this.task.tolerances.settle_dwell_s&&!this.flags.settled){this.flags.settled=true;this.#event('settled',observation,{dwell_s:observation.simulationTimeSeconds-this.settleStartS});}
    }else this.settleStartS=null;

    const pinch=observation.openarm?.pinchReferences?.[this.task.side]?.positionM;
    if(this.flags.settled&&Array.isArray(pinch)){
      const graspPoint=worldPoint(objectBody,object.affordances?.graspReferenceM||[0,0,0]);
      const distance=Math.hypot(...pinch.map((value,index)=>value-graspPoint[index]));
      if(distance>=this.task.tolerances.retreat_m&&!this.flags.retreat){this.flags.retreat=true;this.#event('retreat',observation,{distance_m:distance});}
    }
    const sequence=this.flags.grasp&&this.flags.lift&&this.flags.transport&&this.flags.receivingRegion&&this.flags.support&&this.flags.release&&this.flags.settled&&this.flags.retreat;
    this.success=!this.flags.excessivePenetration&&!this.flags.prohibitedContact&&(sequenceRequired?sequence:placement.inside&&placement.orientationOk&&placement.supported&&this.flags.settled);
    return this.snapshot();
  }
  snapshot(){
    if(!this.task)return{evaluator:'openarm-lab-transfer-v2',status:'no_task',success:false,readOnly:true,hardwareValidated:false,events:[]};
    const initialOnly=this.initialAlreadySatisfied&&!this.flags.grasp;
    return {evaluator:'openarm-lab-transfer-v2',task_id:this.task.id,task_type:this.task.type,
      status:this.success?(initialOnly?'already_satisfied':'success'):this.failure?'failure':initialOnly&&this.task.required_action_sequence?'already_satisfied_no_transfer_credit':'running_or_ready',
      success:this.success,initialAlreadySatisfied:this.initialAlreadySatisfied,sequenceRequired:this.task.required_action_sequence,
      flags:clone(this.flags),events:clone(this.events.slice(-64)),failure:clone(this.failure),
      evidenceSource:'read-only MuJoCo bodies, velocities and named contacts; phase order and dwell are evaluated independently of the executing user program',
      graspDwellRequiredS:.06,hardwareValidated:false};
  }
}
