#!/usr/bin/env python3
"""Native MuJoCo reference for RoboBuddy Phase 5A OpenArm V2.

Setup writes only the declared initial robot qpos/ctrl values. Ordinary task
execution writes actuator controls and advances MuJoCo. Free flask/beaker qpos
are never written in the nominal run. Negative profiles explicitly alter test
parameters before execution and are reported as such.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / 'models' / 'openarm_v2' / 'manipulation.xml'
JOINTS = [
    *(f'openarm_left_joint{i}' for i in range(1, 8)), 'openarm_left_finger_joint1',
    *(f'openarm_right_joint{i}' for i in range(1, 8)), 'openarm_right_finger_joint1',
]
ACTUATORS = [
    *(f'left_joint{i}_ctrl' for i in range(1, 8)), 'left_finger1_ctrl',
    *(f'right_joint{i}_ctrl' for i in range(1, 8)), 'right_finger1_ctrl',
]
INITIAL = {
    **{f'openarm_left_joint{i}': 0.0 for i in range(1, 8)},
    'openarm_left_joint4': math.pi / 2,
    'openarm_left_finger_joint1': 0.65,
    **{f'openarm_right_joint{i}': 0.0 for i in range(1, 8)},
    'openarm_right_joint4': math.pi / 2,
    'openarm_right_finger_joint1': -0.65,
}

def d2r(v): return math.radians(v)
def arm(side, values, finger=None):
    out = {f'openarm_{side}_joint{i+1}': d2r(v) for i, v in enumerate(values)}
    if finger is not None: out[f'openarm_{side}_finger_joint1'] = float(finger)
    return out

LEFT_LIFT = [-0.545, 0, 0, 97.436, 0, -7.941, 0]
RIGHT_LIFT = [0.545, 0, 0, 97.436, 0, 7.941, 0]
LEFT_TRANSFER = [-26.7699, 0, 0, 64.934, 0, -1.6954, 0]
RIGHT_TRANSFER = [26.7699, 0, 0, 64.934, 0, 1.6954, 0]
LEFT_PLACE = [-27.1394, 0, 0, 56.4231, 0, 6.4055, 0]
RIGHT_PLACE = [27.1394, 0, 0, 56.4231, 0, -6.4055, 0]
STAGES = [
    ('settle_initial', {}, 0.40),
    ('left_close', {'openarm_left_finger_joint1': 0.0}, 0.50),
    ('left_lift', arm('left', LEFT_LIFT, 0.0), 1.00),
    ('left_transfer', arm('left', LEFT_TRANSFER, 0.0), 1.20),
    ('left_lower', arm('left', LEFT_PLACE, 0.0), 0.90),
    ('left_release', {'openarm_left_finger_joint1': 0.65}, 0.45),
    ('left_settle', {}, 0.45),
    ('left_retreat', arm('left', LEFT_TRANSFER, 0.65), 0.85),
    ('right_close', {'openarm_right_finger_joint1': 0.0}, 0.50),
    ('right_lift', arm('right', RIGHT_LIFT, 0.0), 1.00),
    ('right_transfer', arm('right', RIGHT_TRANSFER, 0.0), 1.20),
    ('right_lower', arm('right', RIGHT_PLACE, 0.0), 0.90),
    ('right_release', {'openarm_right_finger_joint1': -0.65}, 0.45),
    ('right_settle', {}, 0.45),
    ('right_retreat', arm('right', RIGHT_TRANSFER, -0.65), 0.85),
]

TARGETS = {
    'flask': {'center': np.array([0.608, 0.1535]), 'half': np.array([0.045, 0.045]), 'support': 'left_hotplate', 'initial_z': 1.085, 'geoms': {'flask_body_geom','flask_grip_geom'}, 'fingers': {'left_inner_fingertip','left_outer_fingertip'}},
    'beaker': {'center': np.array([0.608, -0.1535]), 'half': np.array([0.040, 0.040]), 'support': 'right_ring_gauze', 'initial_z': 1.120, 'geoms': {'beaker_grip_geom'}, 'fingers': {'right_inner_fingertip','right_outer_fingertip'}},
}

def name(model, typ, idx):
    return mujoco.mj_id2name(model, typ, int(idx)) or f'id:{idx}'
def ids(model):
    joints = {j: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS}
    acts = {j: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for j,a in zip(JOINTS, ACTUATORS)}
    bodies = {o: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, o) for o in ('flask','beaker','openarm_left_ee_base_link','openarm_right_ee_base_link')}
    free = {o: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{o}_free') for o in ('flask','beaker')}
    assert min(*joints.values(), *acts.values(), *bodies.values(), *free.values()) >= 0
    return joints, acts, bodies, free

def setup(model, data, joints, acts, trial):
    mujoco.mj_resetData(model, data)
    for j, value in INITIAL.items():
        data.qpos[int(model.jnt_qposadr[joints[j]])] = value
        data.ctrl[acts[j]] = value
    # Explicit negative setup interventions only.
    if trial == 'miss-left':
        fj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'flask_free')
        q = int(model.jnt_qposadr[fj]); data.qpos[q+1] += 0.05
    mujoco.mj_forward(model, data)

def contacts(model, data):
    out = []
    for i in range(data.ncon):
        c=data.contact[i]
        out.append((name(model,mujoco.mjtObj.mjOBJ_GEOM,c.geom1), name(model,mujoco.mjtObj.mjOBJ_GEOM,c.geom2)))
    return out

def object_contact_state(model, data, obj):
    spec=TARGETS[obj]; pairs=contacts(model,data)
    touched=set(); support=False
    for a,b in pairs:
        if a in spec['geoms'] and b in spec['fingers']: touched.add(b)
        if b in spec['geoms'] and a in spec['fingers']: touched.add(a)
        if (a in spec['geoms'] and b==spec['support']) or (b in spec['geoms'] and a==spec['support']): support=True
    return touched, support

def run_trial(trial='nominal', timestep=None):
    model=mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    if timestep is not None: model.opt.timestep=float(timestep)
    data=mujoco.MjData(model)
    joints,acts,bodies,free=ids(model)
    # Negative-only parameter profiles.
    if trial == 'weak-grip':
        for key in ('openarm_left_finger_joint1','openarm_right_finger_joint1'):
            aid=acts[key]; model.actuator_forcerange[aid,:]=(-0.02,0.02); model.actuator_forcelimited[aid]=1
    if trial == 'blocked-left':
        aid=acts['openarm_left_joint1']; model.actuator_forcerange[aid,:]=(-0.001,0.001); model.actuator_forcelimited[aid]=1
    setup(model,data,joints,acts,trial)
    eq_types=[int(x) for x in model.eq_type]
    weld_code=int(mujoco.mjtEq.mjEQ_WELD)
    assert weld_code not in eq_types, eq_types
    assert len(eq_types)==2, eq_types
    home_ee={'left':np.array(data.xpos[bodies['openarm_left_ee_base_link']],float).tolist(),'right':np.array(data.xpos[bodies['openarm_right_ee_base_link']],float).tolist()}

    state={o:{'initial':np.array(data.xpos[bodies[o]],float),'max_z':float(data.xpos[bodies[o]][2]),'contact_seen':set(),'lift':False,'carry_anchor':None,'carry':False,'release':False,'support':False,'settle_samples':0,'retreat':False} for o in ('flask','beaker')}
    stage_diag=[]
    for stage_name, targets, seconds in STAGES:
        if trial == 'insufficient-budget' and stage_name == 'left_transfer': break
        actual_targets=dict(targets)
        if trial == 'outside-left' and stage_name in ('left_transfer','left_lower','left_retreat'):
            actual_targets=arm('left', [-8.0,0,0,80.0,0,-2.0,0], 0.0 if stage_name!='left_retreat' else 0.65)
        for j,v in actual_targets.items(): data.ctrl[acts[j]]=float(v)
        n=int(round(seconds/model.opt.timestep)); assert abs(n*model.opt.timestep-seconds)<1e-9
        for _ in range(n):
            mujoco.mj_step(model,data)
            for obj in ('flask','beaker'):
                st=state[obj]; pos=np.array(data.xpos[bodies[obj]],float); touched,support=object_contact_state(model,data,obj)
                st['max_z']=max(st['max_z'],float(pos[2])); st['contact_seen'].update(touched)
                held=bool(touched); bilateral=len(st['contact_seen'] & TARGETS[obj]['fingers'])==2
                if bilateral and held and pos[2] > TARGETS[obj]['initial_z']+0.02:
                    st['lift']=True
                    if st['carry_anchor'] is None: st['carry_anchor']=pos.copy()
                if st['carry_anchor'] is not None and not st['carry']:
                    if not held: st['carry_anchor']=None
                    elif np.linalg.norm(pos[:2]-st['carry_anchor'][:2])>=0.06: st['carry']=True
                inside=np.all(np.abs(pos[:2]-TARGETS[obj]['center'])<=TARGETS[obj]['half'])
                near=abs(pos[2]-TARGETS[obj]['initial_z'])<=0.015
                if st['carry'] and not held and support and inside and near: st['release']=True
                st['support']=support
                qd=int(model.jnt_dofadr[free[obj]]); lin=float(np.linalg.norm(data.qvel[qd:qd+3])); ang=float(np.linalg.norm(data.qvel[qd+3:qd+6]))
                if st['release'] and support and inside and near and lin<=0.035 and ang<=0.8: st['settle_samples']+=1
                else: st['settle_samples']=0
                if st['settle_samples']*model.opt.timestep>=0.20:
                    ee=np.array(data.xpos[bodies[f"openarm_{TARGETS[obj]['fingers'] and ('left' if obj=='flask' else 'right')}_ee_base_link"]],float)
                    if np.linalg.norm(ee-pos)>=0.05: st['retreat']=True
        stage_diag.append({'stage':stage_name,'time':float(data.time),'flask':np.array(data.xpos[bodies['flask']],float).tolist(),'beaker':np.array(data.xpos[bodies['beaker']],float).tolist(),'contacts':contacts(model,data)})

    def metrics(obj):
        st=state[obj]; pos=np.array(data.xpos[bodies[obj]],float); touched,support=object_contact_state(model,data,obj); inside=bool(np.all(np.abs(pos[:2]-TARGETS[obj]['center'])<=TARGETS[obj]['half']))
        bilateral=len(st['contact_seen'] & TARGETS[obj]['fingers'])==2
        return {'bilateralContact':bilateral,'lifted':st['lift'],'carried':st['carry'],'released':st['release'],'support':support,'settled':st['settle_samples']*model.opt.timestep>=0.20,'retreated':st['retreat'],'insideTarget':inside,'maxZM':st['max_z'],'finalPositionM':pos.tolist(),'contactGeoms':sorted(st['contact_seen'])}
    flask=metrics('flask'); beaker=metrics('beaker')
    success=all(flask[k] for k in ('bilateralContact','lifted','carried','released','support','settled','retreated','insideTarget')) and all(beaker[k] for k in ('bilateralContact','lifted','carried','released','support','settled','retreated','insideTarget'))
    blocked_actual=float(data.qpos[int(model.jnt_qposadr[joints['openarm_left_joint1']])])
    return {'trial':trial,'engine':{'version':mujoco.__version__,'timestepSeconds':float(model.opt.timestep)},'model':{'nq':int(model.nq),'nv':int(model.nv),'nu':int(model.nu),'neq':int(model.neq),'hasObjectWeld':False},'homeEeM':home_ee,'metrics':{'success':bool(success),'flask':flask,'beaker':beaker,'blockedLeftJoint1ActualRad':blocked_actual},'stages':stage_diag}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--trial',choices=('nominal','miss-left','weak-grip','blocked-left','outside-left','insufficient-budget'),default='nominal'); ap.add_argument('--timestep',type=float); args=ap.parse_args()
    print(json.dumps(run_trial(args.trial,timestep=args.timestep),indent=2,sort_keys=True))
if __name__=='__main__': main()
