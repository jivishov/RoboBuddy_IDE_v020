import { expect, test } from '@playwright/test';
import { writeFile } from 'node:fs/promises';

test('OpenArm shared geometry and registered WebMCP equipment/program tools use real browser physics', async ({page}, testInfo) => {
  test.setTimeout(240000); const errors=[]; page.on('pageerror',e=>errors.push(String(e)));
  await page.goto('/tests/fixtures/openarm.html'); await page.waitForFunction(()=>window.ready, null, {timeout:60000});
  const alignment = await page.evaluate(() => {
    sim.renderFrame(); const state=sim.getState().observation;
    return [...sim.presentation.bodyGroups].filter(([id])=>id!=='world').map(([id,group])=>({id,actual:group.position.toArray(),expected:[state.bodies[id].positionM[0]*1000,state.bodies[id].positionM[2]*1000,-state.bodies[id].positionM[1]*1000]}));
  });
  for (const a of alignment) a.actual.forEach((v,i)=>expect(v).toBeCloseTo(a.expected[i],6));
  await page.screenshot({path:testInfo.outputPath('contact-aligned-workcell.png')});
  // WebMCP registration is exercised through a test adapter. No claim that CI's
  // Chromium implements the experimental native WebMCP interface is made.
  const names = await page.evaluate(async()=>{
    const {createWebMcpRegistration}=await import('/src/webmcp/register-ide-tools.js');
    window.tools=new Map(); Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool:async(tool,{signal})=>{tools.set(tool.name,tool);signal.addEventListener('abort',()=>tools.delete(tool.name));}}});
    let executionState='idle',access='assist',epoch=0;
    const app={sim:{backend:sim,getState:()=>sim.getState(),getPhysicalSession:()=>sim.getPhysicalSession(),getPhysicalAuthorityToken:()=>sim.getPhysicalAuthorityToken(),applyPhysicalTargets:(...a)=>sim.applyPhysicalTargets(...a)},runToken:0,
      getExecutionState:()=>executionState,beginExecution(){if(executionState!=='idle')return null;executionState='running';return ++this.runToken;},finishExecution(token){if(token===this.runToken)executionState='idle';},cancelExecution(){this.runToken++;executionState='idle';this.openarmAgentProgramActive=false;},resetSimulation:()=>sim.reset(),setStatus:()=>{},renderPanels:()=>{}};
    const facade={app,controlSequence:0,activeControlId:null,assertActive(e){if(e!==epoch||access!=='assist')throw Error('Agent access inactive');},setRegistrationEpoch(e){epoch=e;},getRegistrationContext:()=>({profileId:'openarm',workspaceStatus:'ready',simulationReady:sim.isReady(),simulationMode:'physical_mujoco',workspaceGeneration:1,simulatorEpoch:1}),shouldRegisterMicroduckControl:()=>false};
    window.testApp=app;window.facade=facade;window.registration=createWebMcpRegistration(facade);await registration.setAccess('assist');
    window.callTool=(name,input,signal)=>tools.get(name).execute(input,{signal});return [...tools.keys()];
  });
  for(const name of ['control_openarm_simulation','inspect_openarm_workcell','manage_openarm_workcell','run_openarm_program'])expect(names).toContain(name);
  const builder=await page.evaluate(async()=>{
    const inspection=await callTool('inspect_openarm_workcell',{});
    const input={schema_version:'robobuddy.openarm.equipment.v1',command:'stage',expected_scene_revision:inspection.authority.sceneRevision,equipment:[{id:'button1',kind:'button',position_m:[.57,.32,1.005]},{id:'tray1',kind:'tray',position_m:[.35,-.38,1.005]},{id:'vial1',kind:'vial',position_m:[.35,-.38,1.08]}]};
    const staged=await callTool('manage_openarm_workcell',input);const during=sim.getPhysicalAuthorityToken();const applied=await callTool('manage_openarm_workcell',{schema_version:input.schema_version,command:'apply',stage_id:staged.result.id,acknowledge_reset:true});
    return {inspection,staged,during,applied,after:sim.getWorkcellState()};
  });
  expect(builder.staged.ok).toBe(true);expect(builder.during).toEqual(builder.inspection.authority);expect(builder.applied.ok).toBe(true);
  expect(builder.applied.result.reset).toBe(true);expect(builder.after.authority.sessionId).not.toBe(builder.during.sessionId);expect(builder.after.equipment).toHaveLength(3);
  await page.screenshot({path:testInfo.outputPath('agent-created-equipment.png')});
  const rollback=await page.evaluate(async()=>{
    const before=sim.getState();const staged=await callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'stage',expected_scene_revision:before.authority.sceneRevision,equipment:[{id:'bad',kind:'block',position_m:[.55,.1535,1.10]}]});
    const result=await callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'apply',stage_id:staged.result.id,acknowledge_reset:true});
    const after=sim.getState();await callTool('manage_openarm_workcell',{schema_version:'robobuddy.openarm.equipment.v1',command:'discard'});return {before,after,result};
  });
  expect(rollback.result.ok).toBe(false);expect(rollback.after).toEqual(rollback.before);
  const result=await page.evaluate(async()=>{
    const tool=position_m=>({side:'left',position_m,quaternion_wxyz:[Math.SQRT1_2,0,-Math.SQRT1_2,0]});
    return callTool('run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,segments:[
      {label:'Lift clear of the reference vessel',tool:tool([.554,.1535,1.25]),duration_seconds:4},
      {label:'Move above the created button',tool:tool([.57,.32,1.23]),duration_seconds:4},
      {label:'Close the empty gripper',targets_rad:{openarm_left_finger_joint1:.02},duration_seconds:3},
      {label:'Approach the button',tool:tool([.57,.32,1.06]),duration_seconds:4},
      {label:'Press the passive spring cap',tool:tool([.57,.32,1.05]),duration_seconds:2,wait_for:{type:'equipment_joint',joint_id:'lab_button1_press',minimum:.004,maximum:.006,timeout_seconds:1,dwell_seconds:.06}},
      {label:'Withdraw and verify spring return',tool:tool([.57,.32,1.15]),duration_seconds:2,wait_for:{type:'equipment_joint',joint_id:'lab_button1_press',minimum:0,maximum:.001,timeout_seconds:1,dwell_seconds:.06}},
      {label:'Verify vial rests inside the tray',duration_seconds:.1,wait_for:{type:'supported',object_id:'lab_vial1',support_geom:'lab_tray1_base',timeout_seconds:1}},
    ]});
  });
  await writeFile(testInfo.outputPath('webmcp-equipment-program.json'),JSON.stringify(result,null,2));
  expect(result.ok,JSON.stringify(result)).toBe(true);expect(result.executionStatus).toBe('completed');expect(result.segments.filter(s=>s.condition).every(s=>s.conditionSatisfied)).toBe(true);
  const outcome=await page.evaluate(()=>({state:sim.getState(),workcell:sim.getWorkcellState(),execution:facade.app.getExecutionState()}));
  expect(outcome.execution).toBe('idle');expect(outcome.workcell.equipmentJoints[0].pressed).toBe(false);
  expect(outcome.state.observation.bodies.flask.positionM[0]).toBeCloseTo(.55,3);
  await page.screenshot({path:testInfo.outputPath('robot-equipment-interaction-completed.png')});
  const fail=await page.evaluate(()=>callTool('run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,segments:[{duration_seconds:.1,wait_for:{type:'equipment_joint',joint_id:'lab_button1_press',minimum:.004,maximum:.006,timeout_seconds:.1}}]}));
  expect(fail.ok).toBe(false);expect(fail.reason).toBe('condition-timeout');
  const cancelled=await page.evaluate(async()=>{const abort=new AbortController();const p=callTool('run_openarm_program',{schema_version:'robobuddy.openarm.program.v1',expected_scene_revision:sim.getPhysicalAuthorityToken().sceneRevision,segments:[{duration_seconds:8}]},abort.signal);setTimeout(()=>abort.abort(),30);const result=await p;return {result,state:facade.app.getExecutionState(),owner:facade.activeControlId};});
  expect(cancelled.result.ok).toBe(false);expect(cancelled.state).toBe('idle');expect(cancelled.owner).toBe(null);
  expect(errors,errors.join('\n')).toEqual([]);
});
