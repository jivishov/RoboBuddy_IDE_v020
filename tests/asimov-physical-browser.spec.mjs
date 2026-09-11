import {test,expect} from '@playwright/test';
import {readFile} from 'node:fs/promises';
import {resolve} from 'node:path';
// Local dependency routing is optional and test-only. Production retains the fleet's pinned CDNs.
async function offlineRoutes(page) {
  if(!process.env.ASIMOV_OFFLINE) return;
  await page.route('https://cdn.jsdelivr.net/npm/three@0.180.0/**',async route=>{
    const path=route.request().url().split('three@0.180.0/')[1];
    await route.fulfill({body:await readFile(resolve('node_modules/three',path)),contentType:'text/javascript'});
  });
}
test('Asimov worker/session/bridge: all variants, stale epoch, pause, command bounds',async({page})=>{
 await page.goto('/physics-slice.html');
 const result=await page.evaluate(async()=>{
  const {PhysicsSession}=await import('/src/physics/session.js');
  const {BrowserMuJoCoBackend}=await import('/src/physics/browser-mujoco-backend.js');
  const {ASIMOV_SCENES}=await import('/src/physics/asimov-scene.js');
  const {LivePythonBridge}=await import('/src/runtime/live-python-bridge.js');
  const s=new PhysicsSession(new BrowserMuJoCoBackend({workerUrl:new URL('/src/physics/asimov-mujoco-worker.js',location.href)}),{observationBatchSteps:4});
  const initial=await s.loadScene(structuredClone(ASIMOV_SCENES.mounted));
  const bridge=new LivePythonBridge(s); await bridge.connect(ASIMOV_SCENES.mounted.robotId);
  await bridge.sendAction({left_elbow_joint:1}); await bridge.advance(1);
  const observation=await bridge.getObservation();
  await s.pause(); await s.advanceSteps(100); const paused=await s.getObservation(); await s.resume();
  await s.reset(); let stale=false; try {await bridge.advance(.1);}catch{stale=true;}
  await s.loadScene(structuredClone(ASIMOV_SCENES.drop)); const drop0=await s.getObservation();await s.advanceSteps(30); const drop1=await s.getObservation();
  await s.loadScene(structuredClone(ASIMOV_SCENES.freebase)); await s.advanceSteps(200);const free=await s.getObservation();
  s.dispose();return {elbow:observation.joints.left_elbow_joint.positionRad,clock:observation.simulationTimeSeconds,pausedClock:paused.simulationTimeSeconds,stale,drop:drop0.root.positionM[2]-drop1.root.positionM[2],freeContacts:free.contactCount};
 });
 expect(Math.abs(result.elbow-1)).toBeLessThan(.1);expect(result.pausedClock).toBe(result.clock);expect(result.stale).toBe(true);expect(result.drop).toBeGreaterThan(.05);
});
test('Asimov actual full STL presentation follows observed bodies; renderer cannot advance clock',async({page})=>{
 await offlineRoutes(page); const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('/physics-slice.html');
 const result=await page.evaluate(async()=>{
  const {AsimovPhysicalSimulator}=await import('/src/physics/asimov-physical-simulator.js');
  const {ASIMOV_WORKSPACES}=await import('/src/physics/asimov-workspaces.js');
  const canvas=document.createElement('canvas');canvas.style='width:850px;height:650px';document.body.replaceChildren(canvas);
  const sim=new AsimovPhysicalSimulator(canvas);window.asimovTestSim=sim;
  await sim.setScenario('asimov',structuredClone(ASIMOV_WORKSPACES['asimov-mounted']));
  const start=sim.getState().simulation_time_s;for(let i=0;i<20;i++)sim.renderFrame();
  const end=sim.getState().simulation_time_s;
  await sim.applyPhysicalTargets({left_elbow_joint:1.2,right_shoulder_pitch_joint:.3},{advanceSeconds:1});sim.renderFrame();
  return {start,end,audit:sim.getPresentationAudit(),alignment:sim.getPresentationAlignment(),elbow:sim.getState().joints.left_elbow_joint};
 });
 expect(result.start).toBe(result.end);expect(result.audit.meshCount).toBe(25);expect(result.audit.bodyCount).toBe(26);
 expect(result.alignment.maxBodyErrorMm).toBeLessThan(.001);expect(errors).toEqual([]);
 await page.screenshot({path:process.env.ASIMOV_SCREENSHOT||'test-results/asimov-mounted.png'});
 await page.evaluate(()=>window.asimovTestSim.dispose());
});
test('Asimov IDE selector, live Python and opt-in WebMCP share the visible physics session',async({page})=>{
 test.setTimeout(600000);
 await page.addInitScript(()=>{
  localStorage.setItem('rbide.profile','asimov');localStorage.setItem('rbide.task.asimov','asimov-mounted');
  const registrations=[];
  Object.defineProperty(document,'modelContext',{configurable:true,value:{registerTool(tool){registrations.push(tool);return Promise.resolve();}}});
  window.asimovRegistrations=registrations;
 });
 await page.goto('/?ci=asimov-physical',{waitUntil:'domcontentloaded'});
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 await expect(page.locator('#robotSelect')).toHaveValue('asimov');
 expect(await page.locator('#taskSelect option').count()).toBe(3);
 await expect(page.locator('#simCanvas')).toHaveAttribute('data-asimov-root-mode','fixed-mounted');
 await page.evaluate(async()=>{const app=window.__robobuddyCi.app; await app.run();});
 const python=await page.evaluate(()=>({out:window.__robobuddyCi.app.console,diagnostics:window.__robobuddyCi.app.sim.backend.getState()}));
 expect(python.out.stderr).toBe('');expect(python.out.stdout).toContain('elbow');expect(python.diagnostics.simulation_time_s).toBeGreaterThan(.9);
 await page.locator('#agentAccessControl button[data-agent-access="assist"]').click();
 const agent=await page.evaluate(async()=>{
  const {agentFacade:facade}=window.__robobuddyCi;
  const {executeAsimovPhysicalControl,WEBMCP_ASIMOV_SCHEMA_VERSION}=await import('/src/webmcp/asimov-physical-control.js');
  const epoch=facade.registrationEpoch;
  const command=await executeAsimovPhysicalControl(facade,{schema_version:WEBMCP_ASIMOV_SCHEMA_VERSION,command:'set_joint_targets',targets_rad:{left_elbow_joint:1.2},advance_seconds:.2},null,epoch);
  let forbidden=false;try{await executeAsimovPhysicalControl(facade,{schema_version:WEBMCP_ASIMOV_SCHEMA_VERSION,command:'walk'},null,epoch);}catch{forbidden=true;}
  return {command,forbidden,registered:window.asimovRegistrations.some(t=>t.name==='control_asimov_physical_simulation')};
 });
 expect(agent.registered).toBe(true);expect(agent.forbidden).toBe(true);expect(agent.command.observedState.joints.left_elbow_joint.requested_target_rad).toBe(1.2);
 await page.screenshot({path:'test-results/asimov-ide.png'});
 await page.locator('#taskSelect').selectOption('asimov-freebase');
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 await expect(page.locator('#simCanvas')).toHaveAttribute('data-asimov-root-mode','free-base');
 await page.locator('#robotSelect').selectOption('unitree');
 await expect(page.locator('#statusMessage')).toContainText('Ready',{timeout:120000});
 await expect(page.locator('#simCanvas')).not.toHaveAttribute('data-asimov-root-mode',/.+/);
});
