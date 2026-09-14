import assert from 'node:assert/strict';
import {
  OPENARM_LAB_SCENE_VERSION, OPENARM_LAB_TASK_VERSION, OPENARM_LAB_PROJECT_VERSION,
  stripOpenArmReferenceWorkcellXml, validateLabSceneSpec, validateLabTaskSpec, createLabProject,
} from '../../src/physics/openarm-lab-builder.js';
import { validateOpenArmLabEquipment, compileOpenArmLabEquipment } from '../../src/physics/openarm-lab-equipment.js';
import { OpenArmLabTransferEvaluator } from '../../src/physics/openarm-lab-transfer-evaluator.js';
import { OPENARM_LAB_BUILDER_TASK, loadPatchedScenario, tasksForProfile } from '../../src/task-catalog.js';
import { buildPatchedWorkspace } from '../../src/task-workspace.js';
import { OPENARM_LAB_BUILDER_TOOL_VERSION, createOpenArmWorkcellSchema } from '../../src/webmcp/openarm-workcell.js';

const checks=[];
function check(name,fn){checks.push([name,fn]);}

check('blank-base transform removes only reference task geometry/bodies',()=>{
  const xml=`<mujoco><worldbody>
    <geom name="floor" type="plane"/><geom name="cell_table" type="box"/><geom name="table_leg_1" type="box"/><geom name="mount_column" type="box"/>
    <body name="openarm_mount"><body name="openarm_left_base_link"><geom name="arm_geom" type="box"/></body></body>
    <body name="flask"><freejoint name="flask_free"/><geom name="flask_body_geom" type="cylinder"/></body>
    <body name="beaker"><freejoint name="beaker_free"/><geom name="beaker_grip_geom" type="cylinder"/></body>
  </worldbody></mujoco>`;
  const blank=stripOpenArmReferenceWorkcellXml(xml);
  for(const required of ['name="floor"','name="mount_column"','name="openarm_mount"','name="openarm_left_base_link"','name="arm_geom"'])assert.ok(blank.includes(required),`missing ${required}`);
  for(const forbidden of ['cell_table','table_leg_1','name="flask"','name="beaker"','flask_body_geom','beaker_grip_geom'])assert.ok(!blank.includes(forbidden),`reference dependency survived: ${forbidden}`);
});

const rawScene={
  schema_version:OPENARM_LAB_SCENE_VERSION,id:'synthetic_fixture',
  reference:{mode:'synthetic_fixture',label:'software integration fixture',image_pixels_available:false,dimensions:{known_length_m:.8,description:'declared bench width',source:'source_provided'}},
  assumptions:['Dry rigid-body software fixture; not independent image reconstruction evidence.'],
  assets:[
    {id:'bench',kind:'bench',position_m:[.62,-.02,.90],dimensions_m:[.80,.58,.08],mass_kg:20,dynamic:false,quantity_evidence:{source:'source_provided'}},
    {id:'tray',kind:'tray',position_m:[.49,.12,.98],dimensions_m:[.18,.12,.025],mass_kg:.2,supported_by:'bench',quantity_evidence:{source:'source_provided'}},
    {id:'sample',kind:'vial',position_m:[.50,.12,1.005],dimensions_m:[.025,.025,.055],mass_kg:.05,dynamic:true,supported_by:'tray',quantity_evidence:{source:'source_provided'}},
    {id:'receiver',kind:'receiver',position_m:[.74,.12,.98],dimensions_m:[.12,.10,.04],mass_kg:.2,dynamic:false,supported_by:'bench',quantity_evidence:{source:'source_provided'}},
    {id:'obstacle',kind:'obstacle',position_m:[.62,-.13,.98],dimensions_m:[.06,.06,.10],mass_kg:.2,dynamic:false,supported_by:'bench',quantity_evidence:{source:'source_provided'}},
  ],
};

check('SceneSpec validates explicit provenance/support without welding',()=>{
  const scene=validateLabSceneSpec(rawScene,validateOpenArmLabEquipment);
  assert.equal(scene.assets.length,5);assert.equal(scene.assets.find(a=>a.id==='sample').supported_by,'tray');
  assert.equal(scene.assets.find(a=>a.id==='tray').dynamic,true,'ordinary free-standing tray must default to a free body');
  const compiled=compileOpenArmLabEquipment(scene.assets.map(({supported_by,quantity_evidence,role,...asset})=>asset));
  assert.ok(compiled.xml.includes('<freejoint name="lab_sample_free"/>'),'loose vial must remain a free body');
  assert.ok(compiled.xml.includes('<freejoint name="lab_tray_free"/>'),'free-standing tray must remain a free body');
  assert.ok(!compiled.xml.includes('weld'),'supported_by must not silently weld equipment');
  assert.equal(compiled.records.find(r=>r.id==='receiver').affordances.supportGeometryId,'lab_receiver_base');
});

check('ordinary ancillary equipment defaults free but explicit installed fixtures stay fixed',()=>{
  const normalized=validateOpenArmLabEquipment([
    {id:'tray_a',kind:'tray',position_m:[.4,.2,1],dimensions_m:[.1,.1,.03],mass_kg:.1},
    {id:'rack_a',kind:'rack',position_m:[.4,-.2,1],dimensions_m:[.1,.1,.04],mass_kg:.1},
    {id:'receiver_a',kind:'receiver',position_m:[.65,.2,1],dimensions_m:[.1,.1,.04],mass_kg:.1},
    {id:'obstacle_a',kind:'obstacle',position_m:[.65,-.2,1],dimensions_m:[.05,.05,.08],mass_kg:.1},
    {id:'receiver_fixed',kind:'receiver',position_m:[.75,0,1],dimensions_m:[.1,.1,.04],mass_kg:.1,dynamic:false},
  ]);
  for(const id of ['tray_a','rack_a','receiver_a','obstacle_a'])assert.equal(normalized.find(item=>item.id===id).dynamic,true,id);
  assert.equal(normalized.find(item=>item.id==='receiver_fixed').dynamic,false);
  const xml=compileOpenArmLabEquipment(normalized).xml;
  for(const id of ['tray_a','rack_a','receiver_a','obstacle_a'])assert.ok(xml.includes(`<freejoint name="lab_${id}_free"/>`),id);
  assert.ok(!xml.includes('lab_receiver_fixed_free'));
});

check('component masses sum to declared mass and are volume weighted',()=>{
  const equipment=validateOpenArmLabEquipment([{id:'receiver',kind:'receiver',position_m:[.7,0,1],dimensions_m:[.12,.10,.04],mass_kg:.25,dynamic:false}]);
  const xml=compileOpenArmLabEquipment(equipment).xml;
  const masses=[...xml.matchAll(/ mass="([0-9.eE+-]+)"/g)].map(m=>Number(m[1]));
  assert.ok(masses.length>=5);assert.ok(Math.abs(masses.reduce((a,b)=>a+b,0)-.25)<1e-9);
  assert.ok(new Set(masses.map(v=>v.toPrecision(8))).size>1,'unequal parts must not receive equal mass blindly');
});

const rawTask={schema_version:OPENARM_LAB_TASK_VERSION,id:'transfer_sample',type:'dry_transfer',object_id:'sample',receiver_id:'receiver',side:'left',required_action_sequence:true,tolerances:{position_m:.012,orientation_rad:.35,settle_dwell_s:.2,retreat_m:.06,max_penetration_m:.002}};
check('TaskSpec supports only a resolved dry transfer and rejects fabricated capabilities',()=>{
  const scene=validateLabSceneSpec(rawScene,validateOpenArmLabEquipment);
  const task=validateLabTaskSpec(rawTask,scene);assert.equal(task.supported,true);assert.equal(task.task.object_id,'sample');
  const unsupported=validateLabTaskSpec({...rawTask,id:'heat_sample',type:'heat'},scene);assert.equal(unsupported.supported,false);assert.equal(unsupported.capability,'unsupported');
});

function makeObservation({time,objectPosition,contacts=[],pinch=[.50,.12,1.08],linear=[0,0,0],angular=[0,0,0],quaternion=[1,0,0,0]}){
  const equipment=compileOpenArmLabEquipment(validateOpenArmLabEquipment(rawScene.assets.map(({supported_by,quantity_evidence,role,...asset})=>asset))).records;
  return {simulationTimeSeconds:time,contactsReadable:true,contactCount:contacts.length,contacts,
    bodies:{lab_sample:{positionM:objectPosition,quaternionWxyz:quaternion,linearVelocityMS:linear,angularVelocityRadS:angular},lab_receiver:{positionM:[.74,.12,.98],quaternionWxyz:[1,0,0,0]}},
    openarm:{equipment,pinchReferences:{left:{positionM:pinch}}}};
}
const objectGeom='lab_sample_solid',supportGeom='lab_receiver_base';
const fingerContacts=()=>[
  {geom1Name:'finger_inner_left_collision_00',geom2Name:objectGeom,normalForceN:1,distanceM:-.0001},
  {geom1Name:'finger_outer_left_collision_00',geom2Name:objectGeom,normalForceN:1,distanceM:-.0001},
];
const innerOnly=()=>[{geom1Name:'finger_inner_left_collision_00',geom2Name:objectGeom,normalForceN:1,distanceM:-.0001}];
const supportContact=()=>({geom1Name:supportGeom,geom2Name:objectGeom,normalForceN:.5,distanceM:-.0001});

check('independent evaluator requires ordered grasp/lift/carry/support/release/settle/retreat',()=>{
  const scene=validateLabSceneSpec(rawScene,validateOpenArmLabEquipment);const task=validateLabTaskSpec(rawTask,scene).task;const e=new OpenArmLabTransferEvaluator(scene,task);
  e.observe(makeObservation({time:0,objectPosition:[.50,.12,1.005]}));
  e.observe(makeObservation({time:.1,objectPosition:[.50,.12,1.005],contacts:fingerContacts()}));
  e.observe(makeObservation({time:.2,objectPosition:[.50,.12,1.05],contacts:fingerContacts()}));
  e.observe(makeObservation({time:.3,objectPosition:[.62,.12,1.05],contacts:fingerContacts()}));
  e.observe(makeObservation({time:.4,objectPosition:[.74,.12,.983],contacts:[...fingerContacts(),supportContact()]}));
  e.observe(makeObservation({time:.5,objectPosition:[.74,.12,.983],contacts:[supportContact()],pinch:[.74,.12,1.08]}));
  const result=e.observe(makeObservation({time:.8,objectPosition:[.74,.12,.983],contacts:[supportContact()],pinch:[.74,.12,1.18]}));
  assert.equal(result.success,true,JSON.stringify(result));
  for(const flag of ['grasp','lift','transport','receivingRegion','support','release','settled','retreat'])assert.equal(result.flags[flag],true,flag);
});

check('evaluator cannot bank early support or carry on unilateral contact',()=>{
  const scene=validateLabSceneSpec(rawScene,validateOpenArmLabEquipment);const task=validateLabTaskSpec(rawTask,scene).task;const e=new OpenArmLabTransferEvaluator(scene,task);
  e.observe(makeObservation({time:0,objectPosition:[.50,.12,1.005],contacts:[supportContact()]}));
  e.observe(makeObservation({time:.1,objectPosition:[.50,.12,1.005],contacts:innerOnly()}));
  e.observe(makeObservation({time:.3,objectPosition:[.65,.12,1.06],contacts:innerOnly()}));
  let result=e.snapshot();assert.equal(result.flags.grasp,false);assert.equal(result.flags.transport,false);assert.equal(result.flags.support,false);
  e.observe(makeObservation({time:.4,objectPosition:[.50,.12,1.005],contacts:fingerContacts()}));
  e.observe(makeObservation({time:.5,objectPosition:[.50,.12,1.05],contacts:fingerContacts()}));
  e.observe(makeObservation({time:.6,objectPosition:[.64,.12,1.05],contacts:fingerContacts()}));
  result=e.observe(makeObservation({time:.7,objectPosition:[.74,.12,.983],contacts:fingerContacts()}));
  assert.equal(result.flags.receivingRegion,true);assert.equal(result.flags.support,false,'support seen before receiving phase must not be banked');assert.equal(result.flags.release,false);
});

check('vial placement is yaw-symmetric but rejects inversion/large tilt',()=>{
  const scene=validateLabSceneSpec(rawScene,validateOpenArmLabEquipment);const task=validateLabTaskSpec(rawTask,scene).task;const e=new OpenArmLabTransferEvaluator(scene,task);
  e.observe(makeObservation({time:0,objectPosition:[.50,.12,1.005]}));
  e.observe(makeObservation({time:.1,objectPosition:[.50,.12,1.005],contacts:fingerContacts()}));
  e.observe(makeObservation({time:.2,objectPosition:[.50,.12,1.05],contacts:fingerContacts()}));
  e.observe(makeObservation({time:.3,objectPosition:[.62,.12,1.05],contacts:fingerContacts()}));
  const inverted=e.observe(makeObservation({time:.4,objectPosition:[.74,.12,.983],contacts:[...fingerContacts(),supportContact()],quaternion:[0,1,0,0]}));
  assert.equal(inverted.flags.receivingRegion,false);assert.equal(inverted.flags.support,false);
});

check('already-satisfied state does not receive transfer credit',()=>{
  const scene=validateLabSceneSpec(rawScene,validateOpenArmLabEquipment);const task=validateLabTaskSpec(rawTask,scene).task;const e=new OpenArmLabTransferEvaluator(scene,task);
  const result=e.observe(makeObservation({time:0,objectPosition:[.74,.12,.983],contacts:[supportContact()],pinch:[.9,.12,1.2]}));
  assert.equal(result.initialAlreadySatisfied,true);assert.equal(result.success,false);assert.equal(result.status,'already_satisfied_no_transfer_credit');
});

check('catalog/editor expose a separate empty Lab Builder workspace',async()=>{
  const tasks=tasksForProfile('openarm');assert.ok(tasks.some(t=>t.id===OPENARM_LAB_BUILDER_TASK.id));
  assert.notEqual(tasks[0].id,OPENARM_LAB_BUILDER_TASK.id,'existing OpenArm reference workspace remains the default');
  const scenario=await loadPatchedScenario('openarm',OPENARM_LAB_BUILDER_TASK.id);assert.equal(scenario.labBuilder,true);assert.equal(scenario.portablePython.referenceActions.length,0);
  const files=buildPatchedWorkspace('openarm',scenario);assert.deepEqual(Object.keys(files).sort(),['lab_project.py','main.py','robot_config.py','workcell.py']);
  for(const token of ['hotplate','ring_gauze','referenceActions'])assert.ok(!files['main.py'].includes(token));
});

check('WebMCP Lab Builder schema exposes bounded SceneSpec/project/task lifecycle',()=>{
  const schema=createOpenArmWorkcellSchema();const commands=schema.oneOf.map(branch=>branch.properties.command.const);
  assert.deepEqual(commands,['stage_scene','stage_project','apply','discard','set_task','plan_transfer','export_project']);
  assert.equal(schema.oneOf[0].properties.schema_version.const,OPENARM_LAB_BUILDER_TOOL_VERSION);
  assert.equal(schema.oneOf[1].properties.project.properties.auto_start.const,false);
});

check('project export is explicit, sparse and never auto-starts',()=>{
  const scene=validateLabSceneSpec(rawScene,validateOpenArmLabEquipment);const task=validateLabTaskSpec(rawTask,scene).task;
  const blank=createLabProject({sceneSpec:scene});assert.equal(blank.auto_start,false);assert.equal('task' in blank,false);assert.equal('program' in blank,false);
  const project=createLabProject({sceneSpec:scene,taskSpec:task,program:{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:'example',segments:[]}});
  assert.equal(project.schema_version,OPENARM_LAB_PROJECT_VERSION);assert.equal(project.auto_start,false);assert.equal(project.task.id,'transfer_sample');
});

for(const[name,fn]of checks){await fn();console.log(`PASS ${name}`);}console.log(`OpenArm Lab Builder core: ${checks.length} checks passed`);