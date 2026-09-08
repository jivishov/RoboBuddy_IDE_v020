import { expect, test } from '@playwright/test';
import {
  createProfileControlSchema,
  executeProfileControl,
  getProfileControlDefinition,
  WEBMCP_DIRECT_CONTROL_PROFILES,
  WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION,
  WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,
} from '../src/webmcp/robot-controls.js';

const isPhysical = (profileId) => profileId === 'so101' || profileId === 'openarm';
function makeFacade(profileId) {
  const calls = [];
  const state = {
    epoch: 7, workspaceGeneration: 3, simulatorEpoch: 11, executionState: 'idle', resetCount: 0, status: '', telemetry: {}, bumpAuthorityOnPhysicalCall: false,
    authority: { sessionId: 'physical-session-1', epoch: 5, sceneRevision: profileId === 'openarm' ? 'phase5a-openarm-bimanual-stack-v1' : 'p4-so101-benchmark-transfer-v3', robotId: profileId === 'openarm' ? 'openarm_v2_bimanual' : 'so101_follower', simulationTimeSeconds: 0 },
  };
  const app = {
    getExecutionState: () => state.executionState,
    sim: {
      getPhysicalAuthorityToken: () => isPhysical(profileId) ? { ...state.authority } : null,
      applyPhysicalTargets: async (targetsRad, options = {}) => {
        calls.push({ kind:'physical', targetsRad:{...targetsRad}, options:{...options} });
        if (state.bumpAuthorityOnPhysicalCall) state.authority.epoch += 1;
        const first=Object.keys(targetsRad)[0];
        return { status:'accepted', commandId:'physical-command-1', acceptedTargetsRad:{...targetsRad}, observation:{ simulationTimeSeconds:state.authority.simulationTimeSeconds+Number(options.advanceSeconds||0), joints:{ [first]:{positionRad:Number(targetsRad[first])/8} }, bodies:profileId==='openarm'?{phase5a_flask:{positionM:[0.41954,0.1535,1.0357]}}:{benchmark_block:{positionM:[0.39416,-0.00169,0.234]}}, contactCount:0 }, taskEvaluation:{success:false} };
      },
      applyAction: async (action, options = {}) => { if (options.beforeTick) await options.beforeTick(); calls.push({kind:'action',action:{...action}}); state.telemetry={...state.telemetry,...action}; return true; },
      advanceTime: async (seconds, options = {}) => { if (options.beforeTick) await options.beforeTick(); calls.push({kind:'advance',seconds}); return true; },
    },
    resetSimulation: async () => { state.resetCount += 1; state.telemetry={}; if (isPhysical(profileId)) state.authority.epoch += 1; return true; },
    setStatus:(message)=>{state.status=message;}, renderPanels:()=>{},
    getAgentSnapshot:()=>({ workspaceStatus:'ready', workspaceGeneration:state.workspaceGeneration, profileId, taskId:profileId==='so101'?'so101-physical-block-transfer':profileId==='openarm'?'openarm-04-filtration-workcell':'mock-task', simulatorEpoch:state.simulatorEpoch, simulationMode:isPhysical(profileId)?'physical_mujoco':profileId==='unitree'?'kinematic_pose':'source_plant', simulation:{ executionState:state.executionState,status:state.status,telemetry:{...state.telemetry},contacts:{},problems:[],preparedActionCount:0 } }),
  };
  const facade = {
    app, activeControlId:null, controlSequence:0,
    assertActive:(expectedEpoch)=>{if(expectedEpoch!==state.epoch) throw new Error('stale epoch');},
    getRegistrationContext:()=>({workspaceStatus:'ready',simulationReady:true,profileId,simulationMode:isPhysical(profileId)?'physical_mujoco':profileId==='unitree'?'kinematic_pose':'source_plant',workspaceGeneration:state.workspaceGeneration,simulatorEpoch:state.simulatorEpoch}),
    inspectSimulation:(snapshot)=>({executionState:snapshot.simulation.executionState,status:snapshot.simulation.status,telemetry:snapshot.simulation.telemetry,contacts:snapshot.simulation.contacts}),
  };
  return {facade,state,calls};
}

test('direct WebMCP control includes bounded OpenArm V2, SO-101, LeKiwi, and Unitree G1', () => {
  expect(WEBMCP_DIRECT_CONTROL_PROFILES).toEqual(['openarm','so101','lekiwi','unitree']);
  expect(getProfileControlDefinition(makeFacade('openarm').facade)?.name).toBe('control_openarm_simulation');
  expect(getProfileControlDefinition(makeFacade('microduck').facade)).toBeNull();
  expect(getProfileControlDefinition(makeFacade('so101').facade)?.name).toBe('control_so101_simulation');
  expect(getProfileControlDefinition(makeFacade('lekiwi').facade)?.name).toBe('control_lekiwi_simulation');
  expect(getProfileControlDefinition(makeFacade('unitree').facade)?.name).toBe('control_unitree_g1_simulation');
});

test('OpenArm V2 schema exposes only source-actuated radian joints and excludes passive fingers', () => {
  const schema=createProfileControlSchema('openarm'); const set=schema.oneOf.find((branch)=>branch.properties.command.const==='set_joint_targets');
  expect(set.properties.schema_version.const).toBe(WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION);
  const keys=Object.keys(set.properties.targets_rad.properties);
  expect(keys).toHaveLength(16);
  expect(keys).toContain('openarm_left_joint1'); expect(keys).toContain('openarm_right_joint7'); expect(keys).toContain('openarm_left_finger_joint1'); expect(keys).toContain('openarm_right_finger_joint1');
  expect(keys).not.toContain('openarm_left_finger_joint2'); expect(keys).not.toContain('openarm_right_finger_joint2');
  expect(set.properties.targets_rad.properties.openarm_left_joint1).toMatchObject({minimum:-3.49066,maximum:1.39626});
  expect(set.properties.targets_rad.properties.openarm_right_joint1).toMatchObject({minimum:-1.39626,maximum:3.49066});
  expect(set.properties.max_steps).toMatchObject({minimum:1,maximum:10000});
  expect(set.properties.advance_seconds).toMatchObject({minimum:0,maximum:2});
});

test('OpenArm V2 WebMCP returns accepted target separately from observed MuJoCo state and rejects passive finger commands', async () => {
  const {facade,calls,state}=makeFacade('openarm'); const before={...state.authority};
  const result=await executeProfileControl(facade,'openarm',{schema_version:WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION,command:'set_joint_targets',targets_rad:{openarm_left_joint1:0.3},advance_seconds:0.02,max_steps:200},new AbortController().signal,7);
  expect(calls).toEqual([{kind:'physical',targetsRad:{openarm_left_joint1:0.3},options:{maxSteps:200,advanceSeconds:0.02}}]);
  expect(result).toMatchObject({ok:true,profileId:'openarm',schemaVersion:WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION,acceptedTargetsRad:{openarm_left_joint1:0.3},observedState:{jointsRad:{openarm_left_joint1:0.0375}}});
  expect(result.physicalAuthority).toMatchObject(before);
  await expect(executeProfileControl(facade,'openarm',{schema_version:WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION,command:'set_joint_targets',targets_rad:{openarm_left_finger_joint2:0.2}},new AbortController().signal,7)).rejects.toMatchObject({code:'INVALID_ARGUMENT'});
});

test('SO-101 schema remains versioned and bounded by its executed joint/actuator intersection', () => {
  const schema=createProfileControlSchema('so101'); const set=schema.oneOf.find((branch)=>branch.properties.command.const==='set_joint_targets');
  expect(set.properties.schema_version.const).toBe(WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION);
  const targets=set.properties.targets_rad;
  expect(Object.keys(targets.properties).sort()).toEqual(['elbow_flex','gripper','shoulder_lift','shoulder_pan','wrist_flex','wrist_roll']);
  expect(targets.properties.shoulder_pan).toMatchObject({minimum:-1.91986,maximum:1.91986});
  expect(targets.properties.gripper.minimum).toBe(-0.17453); expect(targets.properties.gripper.maximum).toBe(1.7453292); expect(targets.properties.wrist_roll.maximum).toBe(2.7438473);
  expect(set.properties.max_steps).toMatchObject({minimum:1,maximum:5000});
});

test('SO-101 WebMCP behavior and stale authority checks remain intact', async () => {
  const {facade,calls,state}=makeFacade('so101'); const before={...state.authority};
  const result=await executeProfileControl(facade,'so101',{schema_version:WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,command:'set_joint_targets',targets_rad:{shoulder_pan:0.2},advance_seconds:0.02,max_steps:20},new AbortController().signal,7);
  expect(calls[0]).toEqual({kind:'physical',targetsRad:{shoulder_pan:0.2},options:{maxSteps:20,advanceSeconds:0.02}});
  expect(result.observedState.jointsRad.shoulder_pan).toBe(0.025); expect(result.physicalAuthority).toMatchObject(before);
  const beforeEpoch=state.authority.epoch;
  const reset=await executeProfileControl(facade,'so101',{schema_version:WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,command:'reset'},new AbortController().signal,7);
  expect(reset.physicalAuthority.epoch).toBe(beforeEpoch+1);
  state.bumpAuthorityOnPhysicalCall=true;
  await expect(executeProfileControl(facade,'so101',{schema_version:WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,command:'set_joint_targets',targets_rad:{shoulder_pan:0.1},max_steps:10},new AbortController().signal,7)).rejects.toMatchObject({code:'OPERATION_CANCELLED'});
});

test('OpenArm physical reset may create a newer authority epoch', async () => {
  const {facade,state}=makeFacade('openarm'); const before=state.authority.epoch;
  const reset=await executeProfileControl(facade,'openarm',{schema_version:WEBMCP_OPENARM_PHYSICAL_SCHEMA_VERSION,command:'reset'},new AbortController().signal,7);
  expect(reset.reset).toBe(true); expect(reset.physicalAuthority.epoch).toBe(before+1);
});

test('LeKiwi requires bounded duration for nonzero base velocity and auto-stops', async () => {
  const {facade,calls}=makeFacade('lekiwi');
  await expect(executeProfileControl(facade,'lekiwi',{command:'set_action',action:{'x.vel':0.2}},new AbortController().signal,7)).rejects.toMatchObject({code:'INVALID_ARGUMENT'});
  const result=await executeProfileControl(facade,'lekiwi',{command:'set_action',action:{'x.vel':0.2,'theta.vel':15},duration_ms:40},new AbortController().signal,7);
  expect(calls).toEqual([{kind:'action',action:{'x.vel':0.2,'theta.vel':15}},{kind:'advance',seconds:0.02},{kind:'action',action:{'x.vel':0,'y.vel':0,'theta.vel':0}}]);
  expect(result).toMatchObject({ok:true,profileId:'lekiwi',durationMs:40,baseAutoStopped:true});
});

test('Unitree G1 bounded partial pose and reset remain browser-only', async () => {
  const {facade,state,calls}=makeFacade('unitree');
  const pose=await executeProfileControl(facade,'unitree',{command:'set_action',action:{waist_pitch_joint:8,left_elbow_joint:45}},new AbortController().signal,7);
  expect(calls[0]).toEqual({kind:'action',action:{waist_pitch_joint:8,left_elbow_joint:45}}); expect(pose.simulation.telemetry).toMatchObject({waist_pitch_joint:8,left_elbow_joint:45});
  const reset=await executeProfileControl(facade,'unitree',{command:'reset'},new AbortController().signal,7); expect(state.resetCount).toBe(1); expect(reset).toMatchObject({ok:true,profileId:'unitree',command:'reset',reset:true,hardwareValidated:false});
});
