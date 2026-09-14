export * from './openarm-lab-builder-simulator-base.js';
import { OpenArmLabBuilderSimulator as BaseSimulator } from './openarm-lab-builder-simulator-base.js';
import { OPENARM_LAB_OBSERVATION_PROFILES, OpenArmSyntheticEstimator, syntheticEstimatorAsProgramObservation } from './openarm-lab-observation-profile.js';
import { OPENARM_LAB_EXECUTION_PROFILES, requireOpenArmLabExecutionProfile } from './openarm-lab-execution-profile.js';
import { createLabProject } from './openarm-lab-builder.js';
import { worldPoint } from './openarm-observation.js';

const clone = value => structuredClone(value);
const distance3 = (a, b) => Math.hypot(...a.map((value, index) => value - b[index]));
function worldAabb(asset, body) {
  if (!asset || !body || !Array.isArray(asset.dimensions_m)) return null;
  const [w,d,h] = asset.dimensions_m;
  const corners = [];
  for (const x of [-w/2,w/2]) for (const y of [-d/2,d/2]) for (const z of [0,h]) corners.push(worldPoint(body,[x,y,z]));
  return { min: [0,1,2].map(i=>Math.min(...corners.map(p=>p[i]))), max: [0,1,2].map(i=>Math.max(...corners.map(p=>p[i]))) };
}
function segmentIntersectsExpandedAabb(a,b,box,margin) {
  let lo=0,hi=1;
  for(let axis=0;axis<3;axis+=1){
    const min=box.min[axis]-margin,max=box.max[axis]+margin,delta=b[axis]-a[axis];
    if(Math.abs(delta)<1e-12){if(a[axis]<min||a[axis]>max)return false;continue;}
    let t1=(min-a[axis])/delta,t2=(max-a[axis])/delta;if(t1>t2)[t1,t2]=[t2,t1];lo=Math.max(lo,t1);hi=Math.min(hi,t2);if(lo>hi)return false;
  }
  return true;
}
function carriedRadius(asset, graspReferenceM) {
  const [w,d,h]=asset.dimensions_m, g=graspReferenceM||[0,0,h/2]; let radius=0;
  for(const x of [-w/2,w/2])for(const y of [-d/2,d/2])for(const z of [0,h])radius=Math.max(radius,Math.hypot(x-g[0],y-g[1],z-g[2]));
  return radius+.005;
}

export class OpenArmLabBuilderSimulator extends BaseSimulator {
  constructor(...args) {
    super(...args);
    this.executionProfileId = 'reference_simulation_v1';
    this.observationProfileId = 'simulator_ground_truth';
    this.estimatorSeed = 1;
    this.syntheticEstimator = new OpenArmSyntheticEstimator({ seed: this.estimatorSeed });
    this.lastCommandIssueSimulationTimeS = null;
    this.lastPlanBinding = null;
    this.programValidation = null;
  }
  async setScenario(...args) {
    const result = await super.setScenario(...args);
    this.executionProfileId='reference_simulation_v1';this.observationProfileId='simulator_ground_truth';this.lastCommandIssueSimulationTimeS=null;this.lastPlanBinding=null;this.programValidation=null;
    this.syntheticEstimator.reset(this.estimatorSeed); if(this.lastObservation)this.syntheticEstimator.push(this.lastObservation); return result;
  }
  async reset(...args) {
    const result=await super.reset(...args);this.lastCommandIssueSimulationTimeS=null;this.lastPlanBinding=null;this.programValidation=null;this.syntheticEstimator.reset(this.estimatorSeed);if(this.lastObservation)this.syntheticEstimator.push(this.lastObservation);return result;
  }
  getExecutionProfile(){return clone(requireOpenArmLabExecutionProfile(this.executionProfileId));}
  setExecutionProfile(profileId,{seed=this.estimatorSeed}={}){
    const profile=requireOpenArmLabExecutionProfile(profileId);
    this.executionProfileId=profile.id;
    this.observationProfileId=profile.observation_profile;
    if(!Number.isInteger(seed)||seed<0||seed>0xffffffff)throw new RangeError('Estimator seed must be a uint32');
    this.estimatorSeed=seed>>>0;this.syntheticEstimator.reset(this.estimatorSeed);if(this.lastObservation)this.syntheticEstimator.push(this.lastObservation);
    this.lastCommandIssueSimulationTimeS=null;this.lastPlanBinding=null;this.programValidation=null;
    return{profile:clone(profile),observationProfile:clone(OPENARM_LAB_OBSERVATION_PROFILES[this.observationProfileId]),seed:this.estimatorSeed};
  }
  setObservationProfile(profileId,{seed=this.estimatorSeed}={}){
    if(!OPENARM_LAB_OBSERVATION_PROFILES[profileId])throw new TypeError(`Unknown observation profile ${profileId}`);
    const executionProfileId=profileId==='synthetic_estimator_v1'?'assumed_hardware_interface_v1':'reference_simulation_v1';
    return this.setExecutionProfile(executionProfileId,{seed});
  }
  async #beforeCommand(){
    const profile=requireOpenArmLabExecutionProfile(this.executionProfileId);
    const now=Number(this.lastObservation?.simulationTimeSeconds||0);
    const period=Math.max(0,Number(profile.external_command_period_s||0));
    const delay=Math.max(0,Number(profile.command_transport_delay_s||0));
    const periodWait=this.lastCommandIssueSimulationTimeS==null?0:Math.max(0,this.lastCommandIssueSimulationTimeS+period-now);
    if(periodWait+delay>0)await this.advanceTime(periodWait+delay);
    this.lastCommandIssueSimulationTimeS=Number(this.lastObservation?.simulationTimeSeconds||0);
  }
  async advanceTime(seconds){
    if(this.observationProfileId!=='synthetic_estimator_v1'){
      const observation=await super.advanceTime(seconds);if(observation)this.syntheticEstimator.push(observation);return observation;
    }
    let remaining=Math.round(Number(seconds)*1000);if(!Number.isInteger(remaining)||remaining<0)throw new RangeError('Advance must align to 1 ms');
    let observation=this.lastObservation;
    while(remaining>0){const steps=Math.min(20,remaining);observation=await super.advanceTime(steps*.001);this.syntheticEstimator.push(observation);remaining-=steps;}
    if(remaining===0&&Number(seconds)===0&&this.lastObservation)this.syntheticEstimator.push(this.lastObservation);
    return observation;
  }
  async applyPhysicalTargets(...args){await this.#beforeCommand();const result=await super.applyPhysicalTargets(...args);if(result?.observation)this.syntheticEstimator.push(result.observation);return result;}
  async applyToolTarget(...args){await this.#beforeCommand();const result=await super.applyToolTarget(...args);if(result?.observation)this.syntheticEstimator.push(result.observation);return result;}
  getSensorObservation(){
    if(!this.lastObservation)return{valid:false,reason:'no_observation',profileId:this.observationProfileId,hardwareValidated:false};
    if(this.observationProfileId==='simulator_ground_truth')return{valid:true,profileId:'simulator_ground_truth',observation:clone(this.lastObservation),source:'MuJoCo authoritative state exposed by the reference execution profile',hardwareValidated:false,cameraPerception:false};
    this.syntheticEstimator.push(this.lastObservation);return this.syntheticEstimator.observe(this.lastObservation.simulationTimeSeconds);
  }
  getProgramObservation(){
    if(this.observationProfileId==='simulator_ground_truth')return clone(this.lastObservation);
    return syntheticEstimatorAsProgramObservation(this.getSensorObservation());
  }
  getPlanningObservation(){return this.getProgramObservation();}
  #limitedTaskSummary(){const evaluation=super.getTaskEvaluation();return{evaluator:evaluation.evaluator,status:evaluation.status,success:Boolean(evaluation.success),hiddenGroundTruthScorekeeper:true,hardwareValidated:false};}
  getWorkcellState(){
    const state=super.getWorkcellState();const profile=this.getExecutionProfile();
    if(this.observationProfileId==='simulator_ground_truth')return{...state,observationMode:this.observationProfileId,executionProfile:profile,executionProfiles:clone(OPENARM_LAB_EXECUTION_PROFILES),observationProfiles:clone(OPENARM_LAB_OBSERVATION_PROFILES),estimatorSeed:this.estimatorSeed,generatedPlanBinding:clone(this.lastPlanBinding),programValidation:clone(this.programValidation),preHardwarePackageStatus:'reference profile active; assumed interface profile available for bounded delay/noise sensitivity testing'};
    const sensor=this.getSensorObservation(),programObservation=syntheticEstimatorAsProgramObservation(sensor);
    const equipment=state.equipment.map(record=>{const body=programObservation?.bodies?.[record.bodyId];return{...record,referencePointsWorldM:body?Object.fromEntries(Object.entries(record.affordances||{}).filter(([,v])=>Array.isArray(v)&&v.length===3).map(([key,v])=>[key,worldPoint(body,v)])): {}};});
    return{...state,observationMode:this.observationProfileId,executionProfile:profile,executionProfiles:clone(OPENARM_LAB_EXECUTION_PROFILES),observationProfiles:clone(OPENARM_LAB_OBSERVATION_PROFILES),estimatorSeed:this.estimatorSeed,sensorValidity:sensor?.valid?{valid:true,sourceSimulationTimeS:sensor.sourceSimulationTimeS,deliveredSimulationTimeS:sensor.deliveredSimulationTimeS,ageSeconds:sensor.ageSeconds}:{valid:false,reason:sensor?.reason||'not_ready'},pinchReferences:clone(programObservation?.openarm?.pinchReferences||{}),equipment,bodies:clone(programObservation?.bodies||{}),joints:clone(programObservation?.joints||{}),contacts:[],contactsAvailable:false,contactEstimates:clone(programObservation?.openarm?.contactEstimates||null),taskEvaluation:this.#limitedTaskSummary(),generatedPlanBinding:clone(this.lastPlanBinding),programValidation:clone(this.programValidation),preHardwarePackageStatus:'assumed hardware interface profile active: planner/program condition checks and agent diagnostics use synthetic estimator; hidden evaluator retains MuJoCo ground truth only for scorekeeping/safety'};
  }
  getTelemetry(){
    const observation=this.observationProfileId==='simulator_ground_truth'?this.lastObservation:this.getProgramObservation();if(!observation)return{};
    const result={simulation_time_s:observation.simulationTimeSeconds,observation_profile:this.observationProfileId};for(const[id,joint]of Object.entries(observation.joints||{}))result[`${id}_rad`]=joint.positionRad;for(const record of this.lastObservation?.openarm?.equipment||[]){const position=observation.bodies?.[record.bodyId]?.positionM;if(!Array.isArray(position))continue;position.forEach((value,index)=>{result[`${record.id}_${'xyz'[index]}_m`]=value;});}return result;
  }
  getContacts(){if(this.observationProfileId==='simulator_ground_truth')return super.getContacts();const sensor=this.getSensorObservation();return{contacts_available:false,contact_forces_available:false,contact_estimates:clone(sensor?.contactEstimates||null),task_summary:this.#limitedTaskSummary(),hardware_validated:false};}

  #planningScene(observation){
    const scene=clone(this.sceneSpec);if(!observation)return scene;
    for(const asset of scene.assets||[]){if(!asset.dynamic)continue;const body=observation.bodies?.[`lab_${asset.id}`];if(Array.isArray(body?.positionM)){asset.position_m=[body.positionM[0],body.positionM[1],body.positionM[2]];}}
    return scene;
  }
  #validateCarriedObjectCorridor(program,observation){
    const assets=this.sceneSpec?.assets||[], objectAsset=assets.find(asset=>asset.id===this.taskSpec?.object_id), receiverAsset=assets.find(asset=>asset.id===this.taskSpec?.receiver_id);
    const objectRecord=(this.lastObservation?.openarm?.equipment||[]).find(record=>record.id===this.taskSpec?.object_id);
    if(!objectAsset||!objectRecord||!observation)return{valid:false,reason:'missing_task_object_or_observation'};
    const closeIndex=program.segments.findIndex(segment=>/close for physical grasp/i.test(segment.label||''));
    const releaseIndex=program.segments.findIndex(segment=>/open gripper while receiver supports object/i.test(segment.label||''));
    const carriedTools=program.segments.slice(Math.max(0,closeIndex+1),releaseIndex<0?program.segments.length:releaseIndex).filter(segment=>segment.tool?.position_m).map(segment=>({label:segment.label,position:[...segment.tool.position_m]}));
    const ignored=new Set([objectAsset.id,receiverAsset?.id,objectAsset.supported_by,receiverAsset?.supported_by].filter(Boolean));
    const radius=carriedRadius(objectAsset,objectRecord.affordances?.graspReferenceM);
    const obstacles=[];
    for(const asset of assets){if(ignored.has(asset.id))continue;const body=observation.bodies?.[`lab_${asset.id}`]||this.lastObservation?.bodies?.[`lab_${asset.id}`];const box=worldAabb(asset,body);if(box)obstacles.push({asset,box});}
    for(let index=1;index<carriedTools.length;index+=1)for(const obstacle of obstacles)if(segmentIntersectsExpandedAabb(carriedTools[index-1].position,carriedTools[index].position,obstacle.box,radius))return{valid:false,reason:'carried_object_corridor',segment:[carriedTools[index-1].label,carriedTools[index].label],equipmentId:obstacle.asset.id,clearanceRadiusM:radius};
    return{valid:true,segments:Math.max(0,carriedTools.length-1),obstacleCount:obstacles.length,clearanceRadiusM:radius,observationProfile:this.observationProfileId};
  }
  async #validateRobotPath(program,observation){
    const waypoints=program.segments.filter(segment=>segment.tool?.position_m).map(segment=>({label:segment.label,positionM:[...segment.tool.position_m],...(segment.tool.quaternion_wxyz?{quaternionWxyz:[...segment.tool.quaternion_wxyz]}:{})}));
    if(!waypoints.length)return{valid:true,totalSamples:0,scope:'no Cartesian waypoints'};
    const startJointPositionsRad=Object.fromEntries(Object.entries(observation?.joints||{}).filter(([id,joint])=>id.startsWith(`openarm_${this.taskSpec.side}_joint`)&&!id.includes('finger')).map(([id,joint])=>[id,joint.positionRad]));
    const bodyPoseOverrides=Object.fromEntries(Object.entries(observation?.bodies||{}).filter(([id])=>id.startsWith('lab_')).map(([id,body])=>[id,{positionM:body.positionM,quaternionWxyz:body.quaternionWxyz}]));
    const accepted=await this.session.sendCommand({type:'validate_tool_waypoints',side:this.taskSpec.side,waypoints,ignoreEquipmentIds:[this.taskSpec.object_id],minimumSeparationM:.0015,jointSampleStepRad:.035,startJointPositionsRad,bodyPoseOverrides},{maxSteps:1});
    return clone(accepted?.observation?.openarm?.motionPlan?.validation||{valid:false,reason:'missing_path_validation_result'});
  }

  async planLabTransfer(...args){
    const planningObservation=this.getPlanningObservation();
    if(!planningObservation)return{supported:false,status:'observation_not_ready',reason:`${this.observationProfileId} has not accumulated enough delayed observations; advance simulation and re-observe before planning.`};
    const originalScene=this.sceneSpec;this.sceneSpec=this.#planningScene(planningObservation);let result;
    try{result=super.planLabTransfer(...args);}finally{this.sceneSpec=originalScene;}
    if(!result?.supported||!result.program){this.lastPlanBinding=null;this.programValidation=null;return result;}
    const program=clone(result.program),equipment=this.lastObservation?.openarm?.equipment||[];
    const objectRecord=equipment.find(record=>record.id===this.taskSpec?.object_id),receiverRecord=equipment.find(record=>record.id===this.taskSpec?.receiver_id);
    const objectBodyId=objectRecord?.bodyId||`lab_${this.taskSpec?.object_id}`,supportGeometryId=receiverRecord?.affordances?.supportGeometryId||`lab_${this.taskSpec?.receiver_id}_base`;
    const lowerIndex=program.segments.findIndex(segment=>/lower into receiver/i.test(segment.label||'')),releaseIndex=program.segments.findIndex(segment=>/open and release/i.test(segment.label||''));
    if(lowerIndex>=0)program.segments[lowerIndex].wait_for={type:'supported',object_id:objectBodyId,support_geom:supportGeometryId,timeout_seconds:2,dwell_seconds:.06};
    if(releaseIndex>=0){const release=program.segments[releaseIndex];delete release.wait_for;release.label='Open gripper while receiver supports object';release.duration_seconds=Math.max(1.5,Number(release.duration_seconds)||0);const lowerTool=lowerIndex>=0?program.segments[lowerIndex].tool:null;if(lowerTool?.position_m){const clearance=clone(lowerTool);clearance.position_m[2]+=.025;program.segments.splice(releaseIndex+1,0,{label:'Clear released object with open gripper',tool:clearance,duration_seconds:1.5,wait_for:{type:'released',side:this.taskSpec.side,object_id:objectBodyId,timeout_seconds:2,dwell_seconds:.08}},{label:'Allow supported object to settle',duration_seconds:Math.max(.35,Number(this.taskSpec?.tolerances?.settle_dwell_s||.2)+.15)});}}
    const carriedValidation=this.#validateCarriedObjectCorridor(program,planningObservation);
    if(!carriedValidation.valid){this.programSpec=null;this.lastPlanBinding=null;this.programValidation={origin:'lab_transfer_planner',observationProfile:this.observationProfileId,stateBound:false,taskSpaceClearanceValidated:false,sampledRobotLinkClearanceValidated:false,continuousCollisionGuarantee:false,rejection:clone(carriedValidation)};return{supported:false,reason:`Transfer rejected by carried-object clearance: ${carriedValidation.reason}`,diagnostics:clone(carriedValidation)};}
    const robotValidation=await this.#validateRobotPath(program,planningObservation);
    if(!robotValidation.valid){this.programSpec=null;this.lastPlanBinding=null;this.programValidation={origin:'lab_transfer_planner',observationProfile:this.observationProfileId,stateBound:false,taskSpaceClearanceValidated:true,sampledRobotLinkClearanceValidated:false,continuousCollisionGuarantee:false,rejection:clone(robotValidation)};return{supported:false,reason:`Transfer rejected by sampled robot-link clearance: ${robotValidation.reason}`,diagnostics:clone(robotValidation)};}
    this.programSpec=clone(program);const authority=this.getPhysicalAuthorityToken(),relevantBodyPositionsM={};
    for(const asset of this.sceneSpec?.assets||[]){if(!asset.dynamic&&![this.taskSpec?.object_id,this.taskSpec?.receiver_id].includes(asset.id))continue;const position=planningObservation?.bodies?.[`lab_${asset.id}`]?.positionM;if(Array.isArray(position))relevantBodyPositionsM[asset.id]=[...position];}
    this.lastPlanBinding={sessionId:authority?.sessionId||null,epoch:authority?.epoch??null,sceneRevision:authority?.sceneRevision||null,observationProfileId:this.observationProfileId,executionProfileId:this.executionProfileId,sourceObservationSimulationTimeS:planningObservation?.openarm?.sourceSimulationTimeS??planningObservation?.simulationTimeSeconds??null,relevantBodyPositionsM,positionValidityToleranceM:.01,programKey:JSON.stringify(program)};
    this.programValidation={origin:'lab_transfer_planner',observationProfile:this.observationProfileId,executionProfile:this.executionProfileId,stateBound:true,taskSpaceClearanceValidated:true,carriedObjectValidation:clone(carriedValidation),sampledRobotLinkClearanceValidated:true,robotLinkValidation:clone(robotValidation),continuousCollisionGuarantee:false};
    return{...result,program:clone(program),planner:{...result.planner,observationProfile:this.observationProfileId,executionProfile:this.executionProfileId,stateBinding:clone(this.lastPlanBinding),feedback:'receiver support before release; sustained release after 25 mm clearance; explicit settle dwell',carriedObjectValidation:clone(carriedValidation),sampledRobotLinkValidation:clone(robotValidation),continuousCollisionGuarantee:false}};
  }
  setLabProgramSpec(program){
    const key=JSON.stringify(program);
    if(this.lastPlanBinding&&key===this.lastPlanBinding.programKey){const authority=this.getPhysicalAuthorityToken();if(!authority||authority.sessionId!==this.lastPlanBinding.sessionId||authority.epoch!==this.lastPlanBinding.epoch||authority.sceneRevision!==this.lastPlanBinding.sceneRevision||this.observationProfileId!==this.lastPlanBinding.observationProfileId||this.executionProfileId!==this.lastPlanBinding.executionProfileId)throw new Error('Generated transfer plan is stale because the authoritative session, scene, execution profile, or observation profile changed; replan from the current state.');const currentObservation=this.getPlanningObservation();if(!currentObservation)throw new Error('Generated transfer plan is stale because the selected observation contract is not currently valid; re-observe and replan.');for(const[assetId,plannedPosition]of Object.entries(this.lastPlanBinding.relevantBodyPositionsM||{})){const current=currentObservation?.bodies?.[`lab_${assetId}`]?.positionM;if(!Array.isArray(current)||distance3(current,plannedPosition)>this.lastPlanBinding.positionValidityToleranceM)throw new Error(`Generated transfer plan is stale because ${assetId} moved beyond the declared 10 mm validity tolerance under ${this.observationProfileId}; re-observe and replan.`);}this.programValidation={...(this.programValidation||{}),origin:'lab_transfer_planner',stateBound:true};}
    else{this.lastPlanBinding=null;this.programValidation={origin:'edited_or_external',observationProfile:this.observationProfileId,executionProfile:this.executionProfileId,stateBound:false,taskSpaceClearanceValidated:false,sampledRobotLinkClearanceValidated:false,continuousCollisionGuarantee:false};}
    return super.setLabProgramSpec(program);
  }
  exportLabProject(){const profile=this.getExecutionProfile();return createLabProject({sceneSpec:this.sceneSpec,taskSpec:this.taskSpec,program:this.programSpec,executionProfile:{...profile,estimator_seed:this.estimatorSeed,program_validation:clone(this.programValidation),hardware_validated:false}});}
}
