"""Check a prescribed planar trajectory in a locally supplied MuJoCo scene."""
import argparse
import json
import math
from pathlib import Path

import mujoco as mj
import numpy as np

FIELDS=('t_s','x_m','y_m','yaw_deg','left_deg','right_deg')
def run(scene,trace,dynamics=False):
    m=mj.MjModel.from_xml_path(str(scene)); d=mj.MjData(m)
    rows=[json.loads(line) for line in Path(trace).read_text(encoding='utf-8-sig').splitlines() if line.strip()]
    path=np.array([[row[k] for k in FIELDS] for row in rows],float)
    if len(path)<2 or not np.isfinite(path).all() or min(np.diff(path[:,0]))<=0:
        raise ValueError('Need finite samples with strictly increasing times')
    base=m.joint('base'); q=int(base.qposadr[0]); v=int(base.dofadr[0])
    jaws=[m.joint(n+'_angle') for n in ('jaw_left','jaw_right')]
    names=[m.geom(i).name for i in range(m.ngeom)]
    robot=set(np.flatnonzero(m.geom_contype==2))
    targets=[i for i in range(m.nbody) if m.body(i).name.startswith('target_')]
    contacts={}; checks=0
    def place(p,vel):
        yaw=math.radians(p[3])/2
        d.qpos[q:q+7]=[p[1],p[2],0,math.cos(yaw),0,0,math.sin(yaw)]
        d.qvel[v:v+6]=[vel[1],vel[2],0,0,0,math.radians(vel[3])]
        for i,j in enumerate(jaws,4):
            d.qpos[j.qposadr[0]]=math.radians(p[i]);d.qvel[j.dofadr[0]]=math.radians(vel[i])
    def observe(p):
        nonlocal checks
        mj.mj_forward(m,d);checks+=1
        for c in d.contact:
            a,b=c.geom
            if (a in robot)==(b in robot):continue
            ri,oi=(a,b) if a in robot else (b,a)
            obj=names[oi]
            if obj=='ground':continue
            part=names[ri].rsplit('_',1)[0]
            event=contacts.setdefault((part,obj),dict(part=part,object=obj,t_first_s=float(p[0]),
                first_pose=dict(zip(FIELDS[1:],p[1:].tolist())),max_penetration_mm=0.))
            event['t_last_s']=float(p[0])
            event['max_penetration_mm']=round(max(-1000*float(c.dist),event['max_penetration_mm']),3)
    place(path[0],np.zeros(6));observe(path[0])
    starts={i:d.xpos[i].copy() for i in targets}
    for a,b in zip(path,path[1:]):
        delta=b-a
        travel=np.linalg.norm(delta[1:3])+math.radians(abs(delta[3]))*.32+math.radians(max(abs(delta[4:6])))*.15
        steps=max(1,math.ceil(travel/.001),math.ceil(delta[0]/.002) if dynamics else 1)
        vel=delta/delta[0]
        if dynamics:m.opt.timestep=delta[0]/steps
        for k in range(steps):
            if dynamics:
                place(a+delta*k/steps,vel);mj.mj_step(m,d)
            pose=a+delta*(k+1)/steps;place(pose,vel);observe(pose)
    return dict(mode='prescribed_pose_with_free_objects' if dynamics else 'pose_geometry',
                samples=checks,contacts=list(contacts.values()),
                target_delta_m={m.body(i).name:(d.xpos[i]-starts[i]).round(6).tolist() for i in targets})

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('scene',type=Path);parser.add_argument('trace',type=Path)
    parser.add_argument('--out',required=True,type=Path);parser.add_argument('--dynamics',action='store_true')
    args=parser.parse_args()
    report=run(args.scene,args.trace,args.dynamics)
    args.out.write_text(json.dumps(report,indent=2),encoding='utf8')
    print(f"{report['samples']} poses, {len(report['contacts'])} contact pairs")

