import { expect, test } from '@playwright/test';

const SCENE_VERSION='robobuddy.lab.scene.v1';
const TASK_VERSION='robobuddy.lab.task.v1';
const TOOL_VERSION='robobuddy.openarm.lab-builder.v1';

function baseAssets(extra=[]){
  return [
    {id:'bench',kind:'bench',position_m:[.41,0,.98],dimensions_m:[.82,1.10,.025],mass_kg:20,dynamic:false,quantity_evidence:{source:'source_provided'}},
    {id:'source_tray',kind:'tray',position_m:[.55,.1535,1.005],dimensions_m:[.10,.10,.030],mass_kg:.18,supported_by:'bench',quantity_evidence:{source:'source_provided'}},
    {id:'sample',kind:'vial',position_m:[.55,.1535,1.008],dimensions_m:[.030,.030,.130],mass_kg:.060,dynamic:true,supported_by:'source_tray',quantity_evidence:{source:'source_provided'}},
    {id:'receiver',kind:'receiver',position_m:[.67,.1535,1.005],dimensions_m:[.110,.100,.050],mass_kg:.30,dynamic:false,supported_by:'bench',quantity_evidence:{source:'source_provided'}},
    ...extra,
  ];
}
function scene(id,extra=[]){return{schema_version:SCENE_VERSION,id,reference:{mode:'synthetic_fixture',label:'negative-control fixture',image_pixels_available:false},assumptions:['Software negative-control fixture; not real-photo reconstruction evidence.'],assets:baseAssets(extra)};}

async function openBuilderWithTools(page){
  await page.goto('/tests/fixtures/openarm-lab-builder.html');
  await page.waitForFunction(()=>window.ready,null,{timeout:60000});
  return page.evaluate(async()=>{
    const {createWebMcpRegistration}=await import('/src/webmcp/register-ide-tools.js');
    window.tools=new Map();Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool:async(tool,{signal})=>{tools.set(tool.name,tool);signal.addEventListener('abort',()=>tools.delete(tool.name));}}});
    let executionState='idle',access='assist',epoch=0;
    const app={sim:{backend:sim,getState:()=>sim.getState(),getPhysicalSession:()=>sim.getPhysicalSession(),getPhysicalAuthorityToken:()=>sim.getPhysicalAuthorityToken(),applyPhysicalTargets:(...args)=>sim.applyPhysicalTargets(...args)},runToken:0,
      getExecutionState:()=>executionState,beginExecution(){if(executionState!=='idle')return null;executionState='running';return++this.runToken;},finishExecution(token){if(token===this.runToken)executionState='idle';},cancelExecution(){this.runToken++;executionState='idle';this.openarmAgentProgramActive=false;},resetSimulation:()=>sim.reset(),setStatus:()=>{},renderPanels:()=>{}};
    const facade={app,controlSequence:0,activeControlId:null,assertActive(e){if(e!==epoch||access!=='assist')throw Error('Agent access inactive');},setRegistrationEpoch(e){epoch=e;},getRegistrationContext:()=>({profileId:'openarm',workspaceStatus:'ready',simulationReady:sim.isReady(),simulationMode:'physical_mujoco',workspaceGeneration:1,simulatorEpoch:1}),shouldRegisterMicroduckControl:()=>false};
    window.testApp=app;window.facade=facade;window.registration=createWebMcpRegistration(facade);await registration.setAccess('assist');window.callTool=(name,input,signal)=>tools.get(name).execute(input,{signal});return[...tools.keys()];
  });
}

async function applySceneAndTask(page,sceneSpec){
  return page.evaluate(async({sceneSpec,TOOL_VERSION,TASK_VERSION})=>{
    const inspection=await callTool('inspect_openarm_workcell',{});
    const staged=await callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'stage_scene',expected_scene_revision:inspection.authority.sceneRevision,scene_spec:sceneSpec});
    const applied=await callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'apply',stage_id:staged.result.id,acknowledge_reset:true});
    const task=await callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'set_task',task_spec:{schema_version:TASK_VERSION,id:'transfer_sample',type:'dry_transfer',object_id:'sample',receiver_id:'receiver',side:'left',required_action_sequence:true,tolerances:{position_m:.015,orientation_rad:.35,settle_speed_ms:.035,settle_angular_speed_rads:.8,settle_dwell_s:.2,retreat_m:.05,max_penetration_m:.002}}});
    return{inspection,staged,applied,task,state:sim.getWorkcellState()};
  },{sceneSpec,TOOL_VERSION,TASK_VERSION});
}

test('blocked intermediate corridor is rejected without scene auto-adjustment',async({page})=>{
  test.setTimeout(120000);const errors=[];page.on('pageerror',error=>errors.push(String(error)));await openBuilderWithTools(page);
  const blocker={id:'blocker',kind:'obstacle',position_m:[.67,.225,1.005],dimensions_m:[.060,.030,.160],mass_kg:.30,dynamic:false,supported_by:'bench',role:'placement-corridor obstruction',quantity_evidence:{source:'source_provided'}};
  const authored=await applySceneAndTask(page,scene('blocked_layout',[blocker]));expect(authored.applied.ok,JSON.stringify(authored.applied)).toBe(true);expect(authored.task.ok).toBe(true);
  const before=await page.evaluate(()=>({revision:sim.getPhysicalAuthorityToken().sceneRevision,scene:structuredClone(sim.sceneSpec),program:sim.programSpec}));
  const plan=await page.evaluate(({TOOL_VERSION})=>callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'plan_transfer'}),{TOOL_VERSION});
  expect(plan.ok,JSON.stringify(plan)).toBe(true);expect(plan.result.supported).toBe(false);expect(JSON.stringify(plan.result)).toContain('clearance');
  const after=await page.evaluate(()=>({revision:sim.getPhysicalAuthorityToken().sceneRevision,scene:structuredClone(sim.sceneSpec),program:sim.programSpec,staged:sim.getWorkcellState().staged}));
  expect(after.revision).toBe(before.revision);expect(after.scene).toEqual(before.scene);expect(after.program).toBeNull();expect(after.staged).toBeNull();
  expect(errors,errors.join('\n')).toEqual([]);
});

test('generated plan is rejected when a loose scene body moves materially without a revision change',async({page})=>{
  test.setTimeout(120000);const errors=[];page.on('pageerror',error=>errors.push(String(error)));await openBuilderWithTools(page);
  const drifter={id:'drifter',kind:'block',position_m:[.35,-.35,1.30],dimensions_m:[.030,.030,.030],mass_kg:.04,dynamic:true,role:'free-body freshness control',quantity_evidence:{source:'source_provided'}};
  const authored=await applySceneAndTask(page,scene('stale_plan_layout',[drifter]));expect(authored.applied.ok,JSON.stringify(authored.applied)).toBe(true);expect(authored.task.ok).toBe(true);
  const planned=await page.evaluate(async({TOOL_VERSION})=>{const result=await callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'plan_transfer'});return{result,program:structuredClone(sim.programSpec),authority:sim.getPhysicalAuthorityToken(),binding:sim.getWorkcellState().generatedPlanBinding,drifter:[...sim.lastObservation.bodies.lab_drifter.positionM]};},{TOOL_VERSION});
  expect(planned.result.ok).toBe(true);expect(planned.result.result.supported,JSON.stringify(planned.result)).toBe(true);expect(planned.binding.relevantBodyPositionsM.drifter).toBeTruthy();
  const moved=await page.evaluate(async()=>{await sim.advanceTime(.08);return{authority:sim.getPhysicalAuthorityToken(),drifter:[...sim.lastObservation.bodies.lab_drifter.positionM]};});
  expect(moved.authority.sceneRevision).toBe(planned.authority.sceneRevision);expect(Math.hypot(...moved.drifter.map((value,index)=>value-planned.drifter[index]))).toBeGreaterThan(.01);
  const timeBefore=moved.authority.simulationTimeSeconds;
  const rejected=await page.evaluate(program=>callTool('run_openarm_program',program),planned.program);
  expect(rejected.ok).toBe(false);expect(JSON.stringify(rejected).toLowerCase()).toContain('stale');
  const after=await page.evaluate(()=>sim.getPhysicalAuthorityToken());expect(after.sceneRevision).toBe(planned.authority.sceneRevision);expect(after.simulationTimeSeconds).toBeCloseTo(timeBefore,9);
  expect(errors,errors.join('\n')).toEqual([]);
});

test('exported project reopens without auto-starting its bounded program',async({page})=>{
  test.setTimeout(120000);const errors=[];page.on('pageerror',error=>errors.push(String(error)));await openBuilderWithTools(page);
  const authored=await applySceneAndTask(page,scene('roundtrip_layout'));expect(authored.applied.ok).toBe(true);
  const plan=await page.evaluate(({TOOL_VERSION})=>callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'plan_transfer'}),{TOOL_VERSION});expect(plan.result.supported,JSON.stringify(plan)).toBe(true);
  const exported=await page.evaluate(({TOOL_VERSION})=>callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'export_project'}),{TOOL_VERSION});expect(exported.ok).toBe(true);expect(exported.result.auto_start).toBe(false);
  const reopened=await page.evaluate(async({project,TOOL_VERSION})=>{const inspection=await callTool('inspect_openarm_workcell',{});const staged=await callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'stage_project',expected_scene_revision:inspection.authority.sceneRevision,project});const applied=await callTool('manage_openarm_workcell',{schema_version:TOOL_VERSION,command:'apply',stage_id:staged.result.id,acknowledge_reset:true});return{applied,state:sim.getWorkcellState(),evaluation:sim.getTaskEvaluation(),execution:testApp.getExecutionState()};},{project:exported.result,TOOL_VERSION});
  expect(reopened.applied.ok,JSON.stringify(reopened.applied)).toBe(true);expect(reopened.applied.result.autoStarted).toBe(false);expect(reopened.execution).toBe('idle');expect(reopened.state.taskSpec.id).toBe('transfer_sample');expect(reopened.state.programSpec.segments.length).toBeGreaterThan(7);expect(reopened.evaluation.success).toBe(false);
  expect(errors,errors.join('\n')).toEqual([]);
});
