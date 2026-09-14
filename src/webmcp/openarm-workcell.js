// The existing OpenArm reference-workcell tools remain intact in the base module.
export * from './openarm-workcell-base.js';
import * as base from './openarm-workcell-base.js';
import { conditionMet } from '../physics/openarm-observation.js';
import { OPENARM_LAB_EXECUTION_PROFILE_VERSION, OPENARM_LAB_EXECUTION_PROFILES } from '../physics/openarm-lab-execution-profile.js';
import { WebMcpDomainError } from './agent-facade.js';
import { plain, onlyKeys, invalid, captureOpenArm, assertOpenArmCurrent, sameAuthority, backendFor } from './openarm-physical-control.js';
import { OPENARM_LAB_SCENE_VERSION, OPENARM_LAB_TASK_VERSION, OPENARM_LAB_PROJECT_VERSION } from '../physics/openarm-lab-builder.js';
import { LAB_EQUIPMENT_KINDS } from '../physics/openarm-lab-equipment.js';

export const OPENARM_LAB_BUILDER_TOOL_VERSION = 'robobuddy.openarm.lab-builder.v1';
const EXECUTION_PROFILE_IDS = Object.freeze(Object.keys(OPENARM_LAB_EXECUTION_PROFILES));
const v3 = { type:'array', items:{type:'number'}, minItems:3, maxItems:3 };
const evidenceSchema = { type:'object', properties:{ source:{enum:['source_provided','user_measured','image_estimated','fitted','assumed']}, source_detail:{type:'string',maxLength:200}, uncertainty_m:{type:'number',minimum:0,maximum:1}, assumption:{type:'string',maxLength:500} }, required:['source'], additionalProperties:false };
const assetSchema = { type:'object', properties:{
  id:{type:'string',pattern:'^[a-z][a-z0-9_]{0,23}$'}, label:{type:'string',maxLength:64}, kind:{enum:[...LAB_EQUIPMENT_KINDS]}, position_m:v3, yaw_rad:{type:'number',minimum:-Math.PI,maximum:Math.PI}, dimensions_m:v3, mass_kg:{type:'number',minimum:.005,maximum:100}, dynamic:{type:'boolean'}, supported_by:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, role:{type:'string',maxLength:120}, quantity_evidence:evidenceSchema,
  parts:{type:'array',minItems:1,maxItems:12,items:{type:'object',properties:{shape:{enum:['box','sphere','cylinder']},position_m:v3,dimensions_m:v3,radius_m:{type:'number',minimum:.001,maximum:.3},height_m:{type:'number',minimum:.002,maximum:.6}},required:['shape','position_m'],additionalProperties:false}},
}, required:['id','kind','position_m','quantity_evidence'], additionalProperties:false };
const referenceSchema = { type:'object', properties:{ mode:{enum:['none','external_reference','local_file','synthetic_fixture']}, label:{type:'string',maxLength:120}, content_sha256:{type:'string',pattern:'^[a-f0-9]{64}$'}, image_pixels_available:{type:'boolean'}, notes:{type:'string',maxLength:1000}, dimensions:{type:'object',properties:{known_length_m:{type:'number',exclusiveMinimum:0,maximum:10},description:{type:'string',maxLength:160},source:{enum:['source_provided','user_measured','image_estimated','fitted','assumed']}},required:['known_length_m','description','source'],additionalProperties:false}}, required:['mode'], additionalProperties:false };
const sceneSchema = { type:'object', properties:{ schema_version:{const:OPENARM_LAB_SCENE_VERSION}, id:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, reference:referenceSchema, assets:{type:'array',maxItems:12,items:assetSchema}, assumptions:{type:'array',maxItems:32,items:{type:'string',maxLength:500}}, adaptation_of:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, adaptation_notes:{type:'string',maxLength:1000} }, required:['schema_version','id','reference','assets'], additionalProperties:false };
const taskSchema = { type:'object', properties:{ schema_version:{const:OPENARM_LAB_TASK_VERSION}, id:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, type:{type:'string'}, object_id:{type:'string'}, receiver_id:{type:'string'}, side:{enum:['left','right']}, required_action_sequence:{type:'boolean'}, prohibited_contacts:{type:'array',maxItems:32,items:{type:'string',maxLength:100}}, tolerances:{type:'object',properties:{position_m:{type:'number',minimum:.002,maximum:.08},orientation_rad:{type:'number',minimum:.03,maximum:Math.PI},settle_speed_ms:{type:'number',minimum:.001,maximum:.2},settle_angular_speed_rads:{type:'number',minimum:.01,maximum:3},settle_dwell_s:{type:'number',minimum:.04,maximum:2},retreat_m:{type:'number',minimum:.02,maximum:.25},max_penetration_m:{type:'number',minimum:.0002,maximum:.01}},additionalProperties:false}}, required:['schema_version','id','type','object_id','receiver_id','side'], additionalProperties:false };
const executionProfileSchema = { type:'object', properties:{
  schema_version:{const:OPENARM_LAB_EXECUTION_PROFILE_VERSION}, id:{enum:[...EXECUTION_PROFILE_IDS]}, label:{type:'string',maxLength:160}, observation_profile:{enum:['simulator_ground_truth','synthetic_estimator_v1']}, command_semantics:{type:'string',maxLength:500}, external_command_period_s:{type:'number',minimum:0,maximum:1}, command_transport_delay_s:{type:'number',minimum:0,maximum:1}, internal_controller:{type:'string',maxLength:120}, internal_controller_update_period_s:{type:'number',exclusiveMinimum:0,maximum:1}, effort_limits:{type:'string',maxLength:500}, plant_dynamics:{type:'string',maxLength:700}, evidence:{type:'string',maxLength:700}, estimator_seed:{type:'integer',minimum:0,maximum:4294967295}, hardware_validated:{const:false},
}, required:['schema_version','id','observation_profile','external_command_period_s','command_transport_delay_s','hardware_validated'], additionalProperties:false };
const projectSchema = { type:'object', properties:{ schema_version:{const:OPENARM_LAB_PROJECT_VERSION}, scene:sceneSchema, task:taskSchema, program:base.createOpenArmProgramSchema(), execution_profile:executionProfileSchema, auto_start:{const:false} }, required:['schema_version','scene'], additionalProperties:false };

export function createOpenArmWorkcellSchema() {
  return { type:'object', oneOf:[
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'stage_scene'},expected_scene_revision:{type:'string'},scene_spec:sceneSchema},required:['schema_version','command','expected_scene_revision','scene_spec'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'stage_project'},expected_scene_revision:{type:'string'},project:projectSchema},required:['schema_version','command','expected_scene_revision','project'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'apply'},stage_id:{type:'string'},acknowledge_reset:{const:true}},required:['schema_version','command','stage_id','acknowledge_reset'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'discard'}},required:['schema_version','command'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'set_execution_profile'},execution_profile_id:{enum:[...EXECUTION_PROFILE_IDS]},estimator_seed:{type:'integer',minimum:0,maximum:4294967295}},required:['schema_version','command','execution_profile_id'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'set_task'},task_spec:taskSchema},required:['schema_version','command','task_spec'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'plan_transfer'}},required:['schema_version','command'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'export_project'}},required:['schema_version','command'],additionalProperties:false},
  ] };
}

function isLabBuilder(facade){return backendFor(facade)?.getWorkcellState?.().workspaceMode==='lab_builder';}
export function getOpenArmWorkcellDefinitions(facade){
  if(!isLabBuilder(facade)) return base.getOpenArmWorkcellDefinitions(facade);
  const c=facade.getRegistrationContext();
  if(c.profileId!=='openarm'||c.simulationMode!=='physical_mujoco'||c.workspaceStatus!=='ready'||!c.simulationReady)return[];
  return[
    {name:'inspect_openarm_workcell',title:'Inspect OpenArm Lab Builder',description:'Read the active Lab Builder SceneSpec/TaskSpec, assumptions, selected execution/observation profile, estimated or ground-truth asset poses according to that profile, scene revision, staged candidate, planner validation and evaluator summary. The assumed pre-hardware profile deliberately withholds raw simulator contacts/forces from agent diagnostics.',inputSchema:{type:'object',properties:{},additionalProperties:false},readOnly:true},
    {name:'manage_openarm_workcell',title:'Author OpenArm lab from structured image interpretation',description:`Use ${OPENARM_LAB_BUILDER_TOOL_VERSION}. stage_scene accepts an inspectable ${OPENARM_LAB_SCENE_VERSION} with reference provenance, quantity evidence, bench/equipment/obstacles and support relations. Ordinary equipment is free-standing unless dynamic:false is explicitly authored. set_execution_profile selects either reference simulator ground truth or a declared delayed/noisy assumed pre-hardware interface; switching invalidates state-bound plans. stage_project reopens an exported project but never starts its program. apply is transactional and explicitly resets to a new scene revision; failed compilation/overlap checks leave the active scene intact. set_task freezes the supported dry_transfer evaluator. plan_transfer uses the selected observation profile, carried-object checks and sampled MuJoCo robot-to-authored-lab clearance without mutating physical state. export_project returns the strict scene/task/program/execution profile. No arbitrary XML, code, URLs, evaluator code or physical-state setters.`,inputSchema:createOpenArmWorkcellSchema(),readOnly:false},
    {name:'run_openarm_program',title:'Run bounded OpenArm Lab Builder program',description:`Execute ${base.OPENARM_PROGRAM_VERSION} against the CURRENT approved Lab Builder revision. Planner-generated programs are freshness-checked again at execution. Under the assumed pre-hardware profile, program conditions are decided only from the delayed/noisy selected observation contract; raw MuJoCo contacts remain hidden evaluator/safety evidence. Edited/imported programs remain bounded but do not inherit planner-clearance claims. Human Stop or Agent Assist revocation cancels execution.`,inputSchema:base.createOpenArmProgramSchema(),readOnly:false},
  ];
}
export function inspectOpenArmWorkcell(facade,input,epoch){return base.inspectOpenArmWorkcell(facade,input,epoch);}

function validateProjectShape(project){
  plain(project,'project');onlyKeys(project,['schema_version','scene','task','program','execution_profile','auto_start'],'project');
  if(project.schema_version!==OPENARM_LAB_PROJECT_VERSION)invalid('Unsupported Lab Builder project schema_version');
  if(project.auto_start===true)invalid('Lab Builder project loading never auto-starts a program');
  if(!project.scene)invalid('Project requires scene');
  if(project.execution_profile){plain(project.execution_profile,'execution_profile');onlyKeys(project.execution_profile,Object.keys(executionProfileSchema.properties),'execution_profile');if(project.execution_profile.schema_version!==OPENARM_LAB_EXECUTION_PROFILE_VERSION||!EXECUTION_PROFILE_IDS.includes(project.execution_profile.id)||project.execution_profile.hardware_validated!==false)invalid('Invalid persisted Lab Builder execution profile');}
}
export async function manageOpenArmWorkcell(facade,input,signal,epoch){
  if(!isLabBuilder(facade))return base.manageOpenArmWorkcell(facade,input,signal,epoch);
  plain(input);if(input.schema_version!==OPENARM_LAB_BUILDER_TOOL_VERSION)invalid('Unsupported Lab Builder tool schema_version');
  const allowed={stage_scene:['schema_version','command','expected_scene_revision','scene_spec'],stage_project:['schema_version','command','expected_scene_revision','project'],apply:['schema_version','command','stage_id','acknowledge_reset'],discard:['schema_version','command'],set_execution_profile:['schema_version','command','execution_profile_id','estimator_seed'],set_task:['schema_version','command','task_spec'],plan_transfer:['schema_version','command'],export_project:['schema_version','command']}[input.command];
  if(!allowed)invalid('Unknown Lab Builder command');onlyKeys(input,allowed);
  if(input.command==='stage_project')validateProjectShape(input.project);
  const baseline=captureOpenArm(facade,epoch),app=facade.app,backend=backendFor(facade);
  if(['stage_scene','stage_project'].includes(input.command)&&input.expected_scene_revision!==baseline.authority.sceneRevision)invalid('Scene revision is stale; inspect the Lab Builder again');
  if(input.command==='apply'&&(typeof input.stage_id!=='string'||input.acknowledge_reset!==true))invalid('Applying requires stage_id and acknowledge_reset:true');
  const id=`webmcp-lab-builder-${epoch}-${++facade.controlSequence}`;facade.activeControlId=id;let token=null;
  try{
    if(input.command==='apply'&&app.beginExecution){token=app.beginExecution();if(token==null)throw new Error('Could not acquire Lab Builder execution lease');}
    const guard=()=>assertOpenArmCurrent(facade,baseline,epoch,signal,token);guard();let result;
    if(input.command==='stage_scene')result=backend.stageSceneSpec(input.scene_spec);
    else if(input.command==='stage_project'){
      result=backend.stageSceneSpec(input.project.scene);backend.__pendingLabProject={task:structuredClone(input.project.task||null),program:structuredClone(input.project.program||null),execution_profile:structuredClone(input.project.execution_profile||null)};result={...result,reopenedProject:true,autoStart:false};
    }else if(input.command==='discard'){backend.__pendingLabProject=null;result=backend.discardStagedEquipment();}
    else if(input.command==='set_execution_profile')result=backend.setExecutionProfile(input.execution_profile_id,{seed:input.estimator_seed??backend.estimatorSeed});
    else if(input.command==='apply'){
      result=await backend.applyStagedEquipment(input.stage_id,true,guard);
      const pending=backend.__pendingLabProject;backend.__pendingLabProject=null;
      if(pending?.execution_profile?.id)result.loadedExecutionProfile=backend.setExecutionProfile(pending.execution_profile.id,{seed:pending.execution_profile.estimator_seed??backend.estimatorSeed});
      if(pending?.task){const taskResult=backend.setLabTaskSpec(pending.task);if(!taskResult.supported)throw new Error(taskResult.reason);result.loadedTask=taskResult.task;}
      if(pending?.program){
        backend.lastPlanBinding=null;
        backend.programValidation={origin:'imported_project',observationProfile:backend.observationProfileId,executionProfile:backend.executionProfileId,stateBound:false,taskSpaceClearanceValidated:false,sampledRobotLinkClearanceValidated:false,continuousCollisionGuarantee:false,requiresReplanBeforeClaimedClearance:true};
        backend.setLabProgramSpec(pending.program);result.loadedProgram=structuredClone(pending.program);
      }
      result.autoStarted=false;
    }else if(input.command==='set_task')result=backend.setLabTaskSpec(input.task_spec);
    else if(input.command==='plan_transfer')result=await backend.planLabTransfer();
    else result=backend.exportLabProject();
    assertOpenArmCurrent(facade,baseline,epoch,signal,token,input.command==='apply');app.setStatus?.(`OpenArm Lab Builder: ${input.command}`);app.renderPanels?.();return{ok:true,command:input.command,result,physicalAuthority:app.sim.getPhysicalAuthorityToken(),hardwareValidated:false};
  }catch(error){if(error instanceof WebMcpDomainError)throw error;throw new WebMcpDomainError('LAB_BUILDER_REJECTED',String(error?.message||error).slice(0,500),{retryable:true});}
  finally{if(token!=null)app.finishExecution?.(token);if(facade.activeControlId===id)facade.activeControlId=null;}
}

function selectedTime(observation){const source=Number(observation?.openarm?.sourceSimulationTimeS);return Number.isFinite(source)?source:Number(observation?.simulationTimeSeconds);}
async function runLabBuilderProgram(facade,input,signal,epoch){
  const app=facade.app,backend=backendFor(facade);
  try{backend.setLabProgramSpec(input);}catch(error){const message=String(error?.message||error);if(/stale|validity tolerance|session|scene revision|moved beyond|profile changed|observation contract/i.test(message))throw new WebMcpDomainError('STALE_PLAN',message,{retryable:true,details:{requiresReplan:true}});throw new WebMcpDomainError('INVALID_PROGRAM_STATE',message.slice(0,500),{retryable:true});}
  const baseline=captureOpenArm(facade,epoch);
  // Ground truth is used here only to validate the bounded schema/object identifiers. Runtime
  // condition decisions below use backend.getProgramObservation(), which follows the selected
  // execution profile and can be the delayed/noisy synthetic estimator contract.
  const program=base.validateOpenArmProgram(input,backend.getWorkcellState(),backend.getState().observation);
  const id=`webmcp-lab-program-${epoch}-${++facade.controlSequence}`;facade.activeControlId=id;
  const session=app.sim.getPhysicalSession();const deadline=performance.now()+180000;let token=null,abortReason=null;const completed=[];
  const abort=()=>{abortReason='cancelled';if(sameAuthority(baseline.authority,app.sim.getPhysicalAuthorityToken()))void session.cancelRun('webmcp-lab-program-abort').catch(()=>{});if(token!=null&&app.runToken===token)app.cancelExecution?.('WEBMCP_ABORT');};
  try{
    token=app.beginExecution?.();if(token==null)throw new Error('OpenArm Lab Builder program could not acquire the execution lease');app.openarmAgentProgramActive=true;signal?.addEventListener('abort',abort,{once:true});
    const guard=()=>{assertOpenArmCurrent(facade,baseline,epoch,signal,token);if(performance.now()>deadline)throw new Error('Program exceeded its 180-second wall-time budget');};
    const advance=async seconds=>{let steps=Math.round(seconds/.001);while(steps>0){guard();if(app.getExecutionState()==='paused'){await new Promise(resolve=>setTimeout(resolve,25));continue;}const count=Math.min(20,steps);await backend.advanceTime(count*.001);steps-=count;guard();}};
    const selectedObservation=()=>backend.getProgramObservation?.()||backend.getState().observation;
    guard();
    for(let i=0;i<program.segments.length;i++){
      guard();const segment=program.segments[i],condition=segment.condition;backend.setProgramProgress?.({status:'running',id,index:i+1,segments:program.segments.length,label:segment.label,condition:condition?.type||null,observationProfile:backend.observationProfileId});app.setStatus?.(`OpenArm program ${i+1}/${program.segments.length}: ${segment.label}`);
      const maxSteps=Math.ceil((segment.durationSeconds+(condition?.timeout_seconds||0)+.2)/.001)+100;
      if(segment.targets)await backend.applyPhysicalTargets(segment.targets,{durationSeconds:segment.durationSeconds,maxSteps});
      else if(segment.tool)await backend.applyToolTarget({...segment.tool,durationSeconds:segment.durationSeconds},{maxSteps});
      else{const references=Object.fromEntries(Object.entries(backend.getState().observation.joints).filter(([key,joint])=>joint.targetRad!=null&&!key.endsWith('finger_joint2')).map(([key,joint])=>[key,joint.referenceRad]));await backend.applyPhysicalTargets(references,{durationSeconds:segment.durationSeconds,maxSteps});}
      await advance(segment.durationSeconds);
      const endGround=backend.getState().observation.simulationTimeSeconds;let met=!condition,since=null,lastSelectedTime=null,lastSelected=null;
      while(condition&&!met){
        guard();const observed=selectedObservation();if(observed){const time=selectedTime(observed);if(Number.isFinite(time)&&(lastSelectedTime==null||time>lastSelectedTime+1e-12)){if(lastSelectedTime!=null&&time-lastSelectedTime>.08)since=null;lastSelectedTime=time;lastSelected=observed;if(conditionMet(observed,condition)){since??=time;met=time-since+1e-9>=condition.dwell_seconds;}else since=null;}}
        if(met)break;const ground=backend.getState().observation.simulationTimeSeconds;if(ground>=endGround+condition.timeout_seconds-1e-9)break;await advance(Math.min(.02,Math.max(.001,Math.round((endGround+condition.timeout_seconds-ground)/.001)*.001)));
      }
      const groundObservation=backend.getState().observation,selected=selectedObservation();const record={index:i+1,label:segment.label,simulationTimeSeconds:groundObservation.simulationTimeSeconds,observationProfile:backend.observationProfileId,selectedObservationSourceTimeS:selected?selectedTime(selected):null,condition:condition?.type||null,conditionSatisfied:condition?met:null};completed.push(record);
      if(condition&&!met){backend.setProgramProgress?.({status:'failed',id,...record});app.setStatus?.(`OpenArm program stopped: ${segment.label} did not satisfy ${condition.type}`);return{ok:false,executionStatus:'failed',reason:'condition-timeout',segments:completed,physicalAuthority:app.sim.getPhysicalAuthorityToken(),taskEvaluation:backend.getWorkcellState().taskEvaluation,observationProfile:backend.observationProfileId,executionProfile:backend.executionProfileId,hardwareValidated:false};}
    }
    guard();backend.setProgramProgress?.({status:'completed',id,segments:completed.length,observationProfile:backend.observationProfileId});app.setStatus?.('OpenArm program complete; observed outcome checks recorded');app.renderPanels?.();return{ok:true,executionStatus:'completed',outcome:program.segments.some(segment=>segment.condition)?'requested-observation-conditions-satisfied':'executed-without-custom-outcome-assertions',segments:completed,taskEvaluation:backend.getWorkcellState().taskEvaluation,observationProfile:backend.observationProfileId,executionProfile:backend.executionProfileId,physicalAuthority:app.sim.getPhysicalAuthorityToken(),hardwareValidated:false};
  }catch(error){if(sameAuthority(baseline.authority,app.sim.getPhysicalAuthorityToken()))backend.setProgramProgress?.({status:abortReason||'interrupted',id,completedSegments:completed.length,reason:String(error.message||error).slice(0,240)});if(error instanceof WebMcpDomainError)throw error;throw new WebMcpDomainError('SIMULATION_REJECTED',String(error.message||error).slice(0,360),{retryable:true,details:{completedSegments:completed.length}});}
  finally{signal?.removeEventListener('abort',abort);if(token!=null&&app.runToken===token){app.openarmAgentProgramActive=false;app.finishExecution?.(token);}if(facade.activeControlId===id)facade.activeControlId=null;}
}

export async function runOpenArmProgram(facade,input,signal,epoch){
  if(!isLabBuilder(facade))return base.runOpenArmProgram(facade,input,signal,epoch);
  return runLabBuilderProgram(facade,input,signal,epoch);
}
