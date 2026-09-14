// The existing OpenArm reference-workcell tools remain intact in the base module.
export * from './openarm-workcell-base.js';
import * as base from './openarm-workcell-base.js';
import { WebMcpDomainError } from './agent-facade.js';
import { plain, onlyKeys, invalid, captureOpenArm, assertOpenArmCurrent, backendFor } from './openarm-physical-control.js';
import { OPENARM_LAB_SCENE_VERSION, OPENARM_LAB_TASK_VERSION, OPENARM_LAB_PROJECT_VERSION } from '../physics/openarm-lab-builder.js';
import { LAB_EQUIPMENT_KINDS } from '../physics/openarm-lab-equipment.js';

export const OPENARM_LAB_BUILDER_TOOL_VERSION = 'robobuddy.openarm.lab-builder.v1';
const v3 = { type:'array', items:{type:'number'}, minItems:3, maxItems:3 };
const evidenceSchema = { type:'object', properties:{ source:{enum:['source_provided','user_measured','image_estimated','fitted','assumed']}, source_detail:{type:'string',maxLength:200}, uncertainty_m:{type:'number',minimum:0,maximum:1}, assumption:{type:'string',maxLength:500} }, required:['source'], additionalProperties:false };
const assetSchema = { type:'object', properties:{
  id:{type:'string',pattern:'^[a-z][a-z0-9_]{0,23}$'}, label:{type:'string',maxLength:64}, kind:{enum:[...LAB_EQUIPMENT_KINDS]}, position_m:v3, yaw_rad:{type:'number',minimum:-Math.PI,maximum:Math.PI}, dimensions_m:v3, mass_kg:{type:'number',minimum:.005,maximum:100}, dynamic:{type:'boolean'}, supported_by:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, role:{type:'string',maxLength:120}, quantity_evidence:evidenceSchema,
  parts:{type:'array',minItems:1,maxItems:12,items:{type:'object',properties:{shape:{enum:['box','sphere','cylinder']},position_m:v3,dimensions_m:v3,radius_m:{type:'number',minimum:.001,maximum:.3},height_m:{type:'number',minimum:.002,maximum:.6}},required:['shape','position_m'],additionalProperties:false}},
}, required:['id','kind','position_m','quantity_evidence'], additionalProperties:false };
const referenceSchema = { type:'object', properties:{ mode:{enum:['none','external_reference','local_file','synthetic_fixture']}, label:{type:'string',maxLength:120}, content_sha256:{type:'string',pattern:'^[a-f0-9]{64}$'}, image_pixels_available:{type:'boolean'}, notes:{type:'string',maxLength:1000}, dimensions:{type:'object',properties:{known_length_m:{type:'number',exclusiveMinimum:0,maximum:10},description:{type:'string',maxLength:160},source:{enum:['source_provided','user_measured','image_estimated','fitted','assumed']}},required:['known_length_m','description','source'],additionalProperties:false}}, required:['mode'], additionalProperties:false };
const sceneSchema = { type:'object', properties:{ schema_version:{const:OPENARM_LAB_SCENE_VERSION}, id:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, reference:referenceSchema, assets:{type:'array',maxItems:12,items:assetSchema}, assumptions:{type:'array',maxItems:32,items:{type:'string',maxLength:500}}, adaptation_of:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, adaptation_notes:{type:'string',maxLength:1000} }, required:['schema_version','id','reference','assets'], additionalProperties:false };
const taskSchema = { type:'object', properties:{ schema_version:{const:OPENARM_LAB_TASK_VERSION}, id:{type:'string',pattern:'^[a-z][a-z0-9_]{0,31}$'}, type:{type:'string'}, object_id:{type:'string'}, receiver_id:{type:'string'}, side:{enum:['left','right']}, required_action_sequence:{type:'boolean'}, prohibited_contacts:{type:'array',maxItems:32,items:{type:'string',maxLength:100}}, tolerances:{type:'object',properties:{position_m:{type:'number',minimum:.002,maximum:.08},orientation_rad:{type:'number',minimum:.03,maximum:Math.PI},settle_speed_ms:{type:'number',minimum:.001,maximum:.2},settle_angular_speed_rads:{type:'number',minimum:.01,maximum:3},settle_dwell_s:{type:'number',minimum:.04,maximum:2},retreat_m:{type:'number',minimum:.02,maximum:.25},max_penetration_m:{type:'number',minimum:.0002,maximum:.01}},additionalProperties:false}}, required:['schema_version','id','type','object_id','receiver_id','side'], additionalProperties:false };
const projectSchema = { type:'object', properties:{ schema_version:{const:OPENARM_LAB_PROJECT_VERSION}, scene:sceneSchema, task:taskSchema, program:base.createOpenArmProgramSchema(), execution_profile:{type:'object'}, auto_start:{const:false} }, required:['schema_version','scene'], additionalProperties:false };

export function createOpenArmWorkcellSchema() {
  return { type:'object', oneOf:[
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'stage_scene'},expected_scene_revision:{type:'string'},scene_spec:sceneSchema},required:['schema_version','command','expected_scene_revision','scene_spec'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'stage_project'},expected_scene_revision:{type:'string'},project:projectSchema},required:['schema_version','command','expected_scene_revision','project'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'apply'},stage_id:{type:'string'},acknowledge_reset:{const:true}},required:['schema_version','command','stage_id','acknowledge_reset'],additionalProperties:false},
    {type:'object',properties:{schema_version:{const:OPENARM_LAB_BUILDER_TOOL_VERSION},command:{const:'discard'}},required:['schema_version','command'],additionalProperties:false},
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
    {name:'inspect_openarm_workcell',title:'Inspect OpenArm Lab Builder',description:'Read the active Lab Builder SceneSpec/TaskSpec, assumptions, physical asset poses, contacts, pinch frames, scene revision, staged candidate, planner validation and evaluator state. Simulator ground truth is labelled as such; image pixels are not required when an external multimodal agent supplied structured interpretation.',inputSchema:{type:'object',properties:{},additionalProperties:false},readOnly:true},
    {name:'manage_openarm_workcell',title:'Author OpenArm lab from structured image interpretation',description:`Use ${OPENARM_LAB_BUILDER_TOOL_VERSION}. stage_scene accepts an inspectable ${OPENARM_LAB_SCENE_VERSION} with reference provenance, quantity evidence, bench/equipment/obstacles and support relations. Ordinary equipment is free-standing unless dynamic:false is explicitly authored. stage_project reopens an exported project but never starts its program. apply is transactional and explicitly resets to a new scene revision; failed compilation/overlap checks leave the active scene intact. set_task freezes the supported dry_transfer evaluator. plan_transfer creates an editable state-bound program and performs carried-object plus sampled MuJoCo robot-to-authored-lab clearance validation without mutating the plant. export_project returns scene/task/program together. No arbitrary XML, code, URLs, evaluator code or physical-state setters.`,inputSchema:createOpenArmWorkcellSchema(),readOnly:false},
    {name:'run_openarm_program',title:'Run bounded OpenArm Lab Builder program',description:`Execute ${base.OPENARM_PROGRAM_VERSION} against the CURRENT approved Lab Builder revision. Planner-generated programs are freshness-checked again at execution; edited/external programs remain bounded but do not inherit planner-clearance claims. Application-owned task success remains independent of the user program. Human Stop or Agent Assist revocation cancels execution.`,inputSchema:base.createOpenArmProgramSchema(),readOnly:false},
  ];
}
export function inspectOpenArmWorkcell(facade,input,epoch){return base.inspectOpenArmWorkcell(facade,input,epoch);}

function validateProjectShape(project){
  plain(project,'project');onlyKeys(project,['schema_version','scene','task','program','execution_profile','auto_start'],'project');
  if(project.schema_version!==OPENARM_LAB_PROJECT_VERSION)invalid('Unsupported Lab Builder project schema_version');
  if(project.auto_start===true)invalid('Lab Builder project loading never auto-starts a program');
  if(!project.scene)invalid('Project requires scene');
}
export async function manageOpenArmWorkcell(facade,input,signal,epoch){
  if(!isLabBuilder(facade))return base.manageOpenArmWorkcell(facade,input,signal,epoch);
  plain(input);if(input.schema_version!==OPENARM_LAB_BUILDER_TOOL_VERSION)invalid('Unsupported Lab Builder tool schema_version');
  const allowed={stage_scene:['schema_version','command','expected_scene_revision','scene_spec'],stage_project:['schema_version','command','expected_scene_revision','project'],apply:['schema_version','command','stage_id','acknowledge_reset'],discard:['schema_version','command'],set_task:['schema_version','command','task_spec'],plan_transfer:['schema_version','command'],export_project:['schema_version','command']}[input.command];
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
    else if(input.command==='apply'){
      result=await backend.applyStagedEquipment(input.stage_id,true,guard);
      const pending=backend.__pendingLabProject;backend.__pendingLabProject=null;
      if(pending?.task){const taskResult=backend.setLabTaskSpec(pending.task);if(!taskResult.supported)throw new Error(taskResult.reason);result.loadedTask=taskResult.task;}
      if(pending?.program){backend.setLabProgramSpec(pending.program);result.loadedProgram=structuredClone(pending.program);}
      result.autoStarted=false;
    }else if(input.command==='set_task')result=backend.setLabTaskSpec(input.task_spec);
    else if(input.command==='plan_transfer')result=await backend.planLabTransfer();
    else result=backend.exportLabProject();
    assertOpenArmCurrent(facade,baseline,epoch,signal,token,input.command==='apply');app.setStatus?.(`OpenArm Lab Builder: ${input.command}`);app.renderPanels?.();return{ok:true,command:input.command,result,physicalAuthority:app.sim.getPhysicalAuthorityToken(),hardwareValidated:false};
  }catch(error){if(error instanceof WebMcpDomainError)throw error;throw new WebMcpDomainError('LAB_BUILDER_REJECTED',String(error?.message||error).slice(0,500),{retryable:true});}
  finally{if(token!=null)app.finishExecution?.(token);if(facade.activeControlId===id)facade.activeControlId=null;}
}
export async function runOpenArmProgram(facade,input,signal,epoch){
  if(!isLabBuilder(facade))return base.runOpenArmProgram(facade,input,signal,epoch);
  const backend=backendFor(facade);backend.setLabProgramSpec(input);return base.runOpenArmProgram(facade,input,signal,epoch);
}
