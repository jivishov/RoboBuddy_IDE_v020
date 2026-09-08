#!/usr/bin/env python3
"""Independent native MuJoCo reference for RoboBuddy OpenArm V2 Phase 5A.

Ordinary task execution writes actuator controls only. qpos writes are restricted to
explicit deterministic reset/setup. Objects remain free bodies; no equality/weld is
created or activated for task grasping.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / 'models' / 'openarm_v2' / 'phase5a.xml'
DT = 0.001
INITIAL = {
    'openarm_left_joint1': 0.15, 'openarm_left_joint2': 0.0, 'openarm_left_joint3': 0.0,
    'openarm_left_joint4': math.pi/2, 'openarm_left_joint5': 0.0, 'openarm_left_joint6': 0.0, 'openarm_left_joint7': 0.0,
    'openarm_left_finger_joint1': 0.45, 'openarm_left_finger_joint2': 0.45,
    'openarm_right_joint1': -0.15, 'openarm_right_joint2': 0.0, 'openarm_right_joint3': 0.0,
    'openarm_right_joint4': math.pi/2, 'openarm_right_joint5': 0.0, 'openarm_right_joint6': 0.0, 'openarm_right_joint7': 0.0,
    'openarm_right_finger_joint1': -0.45, 'openarm_right_finger_joint2': -0.45,
}
ACTUATED = {
    'openarm_left_joint1':'left_joint1_ctrl', 'openarm_left_joint2':'left_joint2_ctrl', 'openarm_left_joint3':'left_joint3_ctrl', 'openarm_left_joint4':'left_joint4_ctrl',
    'openarm_left_joint5':'left_joint5_ctrl', 'openarm_left_joint6':'left_joint6_ctrl', 'openarm_left_joint7':'left_joint7_ctrl', 'openarm_left_finger_joint1':'left_finger1_ctrl',
    'openarm_right_joint1':'right_joint1_ctrl', 'openarm_right_joint2':'right_joint2_ctrl', 'openarm_right_joint3':'right_joint3_ctrl', 'openarm_right_joint4':'right_joint4_ctrl',
    'openarm_right_joint5':'right_joint5_ctrl', 'openarm_right_joint6':'right_joint6_ctrl', 'openarm_right_joint7':'right_joint7_ctrl', 'openarm_right_finger_joint1':'right_finger1_ctrl',
}
STAGES = [
    ('settle_initial',0.20,{}),
    ('left_approach',0.60,{'openarm_left_joint1':0.30,'openarm_left_joint2':0.0}),
    ('left_close',0.40,{'openarm_left_finger_joint1':0.0}),
    ('left_lift',0.70,{'openarm_left_joint1':0.15,'openarm_left_joint2':-0.20}),
    ('left_transfer',0.80,{'openarm_left_joint1':0.0,'openarm_left_joint2':-0.40}),
    ('left_lower',0.60,{'openarm_left_joint1':0.275,'openarm_left_joint2':-0.40}),
    ('left_release',0.40,{'openarm_left_finger_joint1':0.45}),
    ('left_settle',0.40,{}),
    ('left_retreat',0.40,{'openarm_left_joint1':0.0,'openarm_left_joint2':-0.40}),
    ('right_approach',0.60,{'openarm_right_joint1':-0.30,'openarm_right_joint2':0.0}),
    ('right_close',0.40,{'openarm_right_finger_joint1':0.0}),
    ('right_lift',0.70,{'openarm_right_joint1':-0.15,'openarm_right_joint2':0.20}),
    ('right_transfer',0.80,{'openarm_right_joint1':0.0,'openarm_right_joint2':0.40}),
    ('right_lower',0.60,{'openarm_right_joint1':-0.10,'openarm_right_joint2':0.40}),
    ('right_release',0.40,{'openarm_right_finger_joint1':-0.45}),
    ('right_settle',0.40,{}),
    ('right_retreat',0.40,{'openarm_right_joint1':0.0,'openarm_right_joint2':0.40}),
    ('final_settle',0.40,{}),
]
OBJECTS = {
    'flask': dict(body='phase5a_flask', free='phase5a_flask_free', geom='phase5a_flask_geom', fingers=('left_inner_finger_pad','left_outer_finger_pad'), support='hotplate_top', ee='openarm_left_ee_base_link', source=[0.41954,0.1535,1.0357], target=[0.4318,0.2397], support_z=1.028),
    'beaker': dict(body='phase5a_beaker', free='phase5a_beaker_free', geom='phase5a_beaker_geom', fingers=('right_inner_finger_pad','right_outer_finger_pad'), support='wire_gauze_support', ee='openarm_right_ee_base_link', source=[0.41954,-0.1535,1.0357], target=[0.4771,-0.2397], support_z=1.075),
}
HALF_Z=0.0307; TARGET_HALF=np.array([0.03,0.03]); LIFT=0.025; CARRY=0.045


def objid(model, typ, name):
    value=mujoco.mj_name2id(model, typ, name)
    if value < 0: raise RuntimeError(f'missing {name}')
    return value

def geom_name(model, gid): return mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(gid)) or f'geom:{gid}'
def contact_pairs(model,data):
    return {tuple(sorted((geom_name(model,data.contact[i].geom1), geom_name(model,data.contact[i].geom2)))) for i in range(data.ncon)}
def pair(pairs,a,b): return tuple(sorted((a,b))) in pairs

def setup(model,data):
    mujoco.mj_resetData(model,data)
    for joint,value in INITIAL.items():
        jid=objid(model,mujoco.mjtObj.mjOBJ_JOINT,joint); data.qpos[int(model.jnt_qposadr[jid])]=value
        if joint in ACTUATED:
            aid=objid(model,mujoco.mjtObj.mjOBJ_ACTUATOR,ACTUATED[joint]); data.ctrl[aid]=value
    mujoco.mj_forward(model,data)

def state_ids(model):
    out={}
    for key,d in OBJECTS.items():
        bid=objid(model,mujoco.mjtObj.mjOBJ_BODY,d['body']); fid=objid(model,mujoco.mjtObj.mjOBJ_JOINT,d['free']); eid=objid(model,mujoco.mjtObj.mjOBJ_BODY,d['ee'])
        out[key]=(bid,int(model.jnt_dofadr[fid]),eid)
    return out

def object_state(model,data,ids,key):
    bid,dof,eid=ids[key]; pos=np.array(data.xpos[bid],float); qv=np.array(data.qvel[dof:dof+6],float); ee=np.array(data.xpos[eid],float); pairs=contact_pairs(model,data); d=OBJECTS[key]
    grip=any(pair(pairs,d['geom'],f) for f in d['fingers']); support=pair(pairs,d['geom'],d['support'])
    return dict(position=pos, linear=float(np.linalg.norm(qv[:3])), angular=float(np.linalg.norm(qv[3:])), ee=ee, grip=grip, support=support, pairs=[list(x) for x in sorted(pairs)])

def apply_trial(model, trial):
    if trial=='weak-grip':
        for name in ('left_finger1_ctrl','right_finger1_ctrl'):
            aid=objid(model,mujoco.mjtObj.mjOBJ_ACTUATOR,name); model.actuator_forcelimited[aid]=1; model.actuator_forcerange[aid,:]=(-0.001,0.001)
    if trial=='blocked-left':
        aid=objid(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'left_joint1_ctrl'); model.actuator_forcelimited[aid]=1; model.actuator_forcerange[aid,:]=(-0.001,0.001)

def trial_stages(trial):
    rows=[(n,s,dict(t)) for n,s,t in STAGES]
    for i,(n,s,t) in enumerate(rows):
        if trial=='miss' and n=='left_approach': rows[i]=(n,s,{**t,'openarm_left_joint1':0.55})
        if trial=='outside' and n=='left_lower': rows[i]=(n,s,{**t,'openarm_left_joint1':0.45})
        if trial=='loss' and n=='left_transfer': rows[i]=(n,s,{**t,'openarm_left_finger_joint1':0.45})
        if trial=='short-budget' and n in ('left_transfer','left_lower'): rows[i]=(n,0.02,t)
    return rows

def run_trial(trial='nominal', timestep=None):
    model=mujoco.MjModel.from_xml_path(str(MODEL_PATH)); data=mujoco.MjData(model)
    if timestep is not None: model.opt.timestep=float(timestep)
    apply_trial(model,trial); setup(model,data); ids=state_ids(model)
    track={k:dict(contact=False,lift=False,carry=False,release=False,support=False,settled=False,retreat=False,anchor=None,release_dist=None,max_held=0.0,settle_start=None,settle_anchor=None,max_drift=0.0) for k in OBJECTS}
    order_violation=False; diagnostics=[]
    def observe(key):
        nonlocal order_violation
        d=OBJECTS[key]; t=track[key]; st=object_state(model,data,ids,key); pos=st['position']
        if key=='beaker' and not (track['flask']['settled'] and track['flask']['retreat']) and st['grip']: order_violation=True
        enabled= key=='flask' or (track['flask']['settled'] and track['flask']['retreat']) or t['contact']
        if not enabled: return st
        if st['grip']: t['contact']=True
        if st['grip'] and pos[2] > d['source'][2]+LIFT:
            t['lift']=True
            if t['anchor'] is None: t['anchor']=pos.copy()
        if t['anchor'] is not None and not t['carry']:
            if not st['grip']: t['anchor']=None; t['max_held']=0
            else:
                dist=float(np.linalg.norm(pos[:2]-t['anchor'][:2])); t['max_held']=max(t['max_held'],dist); t['carry']=dist>=CARRY
        if t['carry'] and not st['grip'] and not t['release']:
            t['release']=True; t['release_dist']=float(np.linalg.norm(st['ee']-pos))
        if t['release'] and st['support']: t['support']=True
        in_target=bool(np.all(np.abs(pos[:2]-np.array(d['target'])) <= TARGET_HALF)); near=abs(pos[2]-(d['support_z']+HALF_Z)) <= 0.012
        rest=t['release'] and st['support'] and not st['grip'] and in_target and near and st['linear']<=0.03 and st['angular']<=0.8
        if not rest: t['settle_start']=None; t['settle_anchor']=None; t['settled']=False; t['max_drift']=0
        elif t['settle_start'] is None: t['settle_start']=float(data.time); t['settle_anchor']=pos.copy()
        else:
            drift=float(np.linalg.norm(pos-t['settle_anchor']))
            if drift>0.001: t['settle_start']=float(data.time); t['settle_anchor']=pos.copy(); t['max_drift']=0; t['settled']=False
            else: t['max_drift']=max(t['max_drift'],drift); t['settled']=float(data.time)-t['settle_start']>=0.20
        if t['settled'] and t['release_dist'] is not None: t['retreat']=float(np.linalg.norm(st['ee']-pos)) >= t['release_dist']+0.025
        t['in_target']=in_target; t['last']=st
        return st
    for name,seconds,targets in trial_stages(trial):
        for joint,value in targets.items(): data.ctrl[objid(model,mujoco.mjtObj.mjOBJ_ACTUATOR,ACTUATED[joint])]=float(value)
        steps=int(round(seconds/model.opt.timestep));
        for step in range(steps):
            mujoco.mj_step(model,data)
            if step%10==0 or step==steps-1:
                observe('flask'); observe('beaker')
        diagnostics.append(dict(stage=name,time=float(data.time),flask={k:v for k,v in object_state(model,data,ids,'flask').items() if k!='pairs'},beaker={k:v for k,v in object_state(model,data,ids,'beaker').items() if k!='pairs'}))
    observe('flask'); observe('beaker')
    metrics={}
    for key in OBJECTS:
        t=track[key]; st=t.get('last') or object_state(model,data,ids,key)
        metrics[key]={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in t.items() if k not in ('anchor','settle_anchor','last')}
        metrics[key].update(finalPositionM=st['position'].tolist(),finalLinearSpeed=st['linear'],finalAngularSpeed=st['angular'],finalGrip=st['grip'],finalSupport=st['support'])
    success=(not order_violation and all(all(metrics[k].get(flag,False) for flag in ('contact','lift','carry','release','support','settled','retreat','in_target')) and not metrics[k]['finalGrip'] and metrics[k]['finalSupport'] for k in OBJECTS))
    blocked_error=None
    if trial=='blocked-left':
        jid=objid(model,mujoco.mjtObj.mjOBJ_JOINT,'openarm_left_joint1'); blocked_error=abs(float(data.qpos[int(model.jnt_qposadr[jid])]) - 0.30)
    return dict(trial=trial,success=bool(success),orderViolation=order_violation,engine=dict(version=mujoco.__version__,timestepSeconds=float(model.opt.timestep),integrator=int(model.opt.integrator)),metrics=metrics,blockedLeftTargetErrorRad=blocked_error,diagnostics=diagnostics)

def main():
    p=argparse.ArgumentParser(); p.add_argument('--trial',choices=('nominal','miss','weak-grip','blocked-left','outside','loss','short-budget'),default='nominal'); p.add_argument('--timestep',type=float); a=p.parse_args(); print(json.dumps(run_trial(a.trial,timestep=a.timestep),indent=2,sort_keys=True))
if __name__=='__main__': main()
