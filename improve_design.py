"""Compare phone-biped design revisions with reproducible MuJoCo experiments.
Positive actuator work and torque-squared integrals are effort metrics, not battery Wh.
Run: python improve_design.py
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import xml.etree.ElementTree as ET
import math,json,csv
import numpy as np
import mujoco
import simulate_robot as base

OUT=Path(__file__).resolve().parent
def design_xml(design='baseline',phone=.25,mu=.8,dt=.002,spring=0.,kp=22.,kv=.65):
    root=ET.fromstring(base.model_xml(phone,mu))
    root.find('option').set('timestep',str(dt))
    if design!='baseline':
        # Shorter legs; landscape phone reduces top height without lowering it into deck.
        for body in root.findall('.//body'):
            n=body.get('name','')
            if n.endswith('_shin') or n.endswith('_ankle_pitch'):
                body.set('pos','0 0 -.070')
        for g in root.findall('.//geom'):
            if g.get('pos')=='0 0 -.040':
                g.set('pos','0 0 -.035')
                dims=g.get('size').split();dims[2]='.035';g.set('size',' '.join(dims))
                g.set('mass','.085' if g.get('mass')=='.100' else '.060')
        for s,y in [('left',.059),('right',-.059)]:
            root.find(f'.//body[@name="{s}_yaw"]').set('pos',f'0 {y} -.012')
            sole=root.find(f'.//geom[@name="{s}_sole"]')
            sole.set('size','.060 .036 .006');sole.set('mass','.095')
            for g in root.find(f'.//body[@name="{s}_foot"]').findall('geom'):
                if g.get('name')!=f'{s}_sole':g.set('size','.055 .033 .002');g.set('mass','.019')
        battery=root.find('.//geom[@name="battery"]')
        battery.set('pos','0 0 -.012');battery.set('size','.037 .032 .018')
        cradle=root.find('.//body[@name="cradle"]/geom')
        cradle.set('pos','-.008 0 .010');cradle.set('size','.004 .077 .036')
        phonebody=root.find('.//body[@name="phone"]');phonebody.set('pos','0 0 .010')
        root.find('.//geom[@name="phone_body"]').set('size','.0045 .073 .034')
        root.find('.//geom[@name="screen"]').set('size','.0003 .066 .030')
    for p in root.findall('./actuator/position'):
        if not p.get('name').startswith('neck'):
            p.set('kp',str(kp));p.set('kv',str(kv))
    if spring:
        for j in root.findall('.//joint'):
            if j.get('name','').endswith('_knee'):
                j.set('stiffness',str(spring));j.set('springref','0')
    return ET.tostring(root,encoding='unicode')

def experiment(design='baseline',gait=1.,period=.95,kp=22.,kv=.65,spring=0.,feedback=0.,scenario='step',push=12.,mu=.8,phone=.25,dt=.002,duration=6.,record=False):
    xml=design_xml(design,phone,mu,dt,spring,kp,kv)
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    soles=[m.geom(s+'_sole').id for s in ['left','right']];ground=m.geom('ground').id
    d.qpos[2]+=.007-min(d.geom_xpos[s,2] for s in soles);mujoco.mj_forward(m,d)
    pelvis=m.body('pelvis').id
    initial_x=float(d.xpos[pelvis,0]);com=float(d.subtree_com[pelvis,2]);height=max(d.geom_xpos[g,2]+m.geom_size[g,2] for g in [m.geom('phone_body').id])
    aid={m.actuator(i).name:i for i in range(m.nu)}
    ndof=[m.jnt_dofadr[m.actuator_trnid[i,0]] for i in range(m.nu)]
    positive_work=0.;tau2=0.;peak=0.;max_tilt=0.;fall=None;saturated=0;rows=[];frames=[];task10=None;slip=0.
    initial_z=float(d.xpos[pelvis,2]);single=0;active_duration=0.
    for i in range(round(duration/dt)):
        t=float(d.time)
        q=base.targets(0,'stand')
        if scenario=='step' and t>1:
            ramp=min((t-1)/.6,1.);phase=(t-1)*2*math.pi/period
            for s,offset in [('left',0),('right',math.pi)]:
                lift=max(0,math.sin(phase+offset))*ramp*gait
                pitch=.13*math.cos(phase+offset)*ramp*gait-.20*lift
                q[s+'_hip_pitch']=pitch;q[s+'_knee']=.48*lift;q[s+'_ankle_pitch']=-pitch-.48*lift
                q[s+'_hip_roll']=.065*math.sin(phase)*ramp*gait;q[s+'_ankle_roll']=-q[s+'_hip_roll']
        if scenario=='stand':q['neck_yaw']=.48*math.sin(max(0,t-1)*1.1)
        if feedback:
            R=d.xmat[pelvis].reshape(3,3)
            pitch=math.atan2(R[0,2],R[2,2]);roll=math.atan2(-R[1,2],R[2,2])
            for s in ['left','right']:
                q[s+'_ankle_pitch']+=float(np.clip(feedback*(pitch+.07*d.cvel[pelvis,1]),-.25,.25))
                q[s+'_ankle_roll']+=float(np.clip(feedback*(roll+.07*d.cvel[pelvis,0]),-.20,.20))
        d.ctrl[:]=[q[n] for n in aid]
        d.xfrc_applied[:]=0
        if scenario.startswith('push') and 2<=t<2.16:
            axis=1 if scenario=='push_side' else 0
            d.xfrc_applied[pelvis,axis]=push
        mujoco.mj_step(m,d)
        tilt=math.degrees(math.acos(float(np.clip(d.xmat[pelvis].reshape(3,3)[2,2],-1,1))))
        max_tilt=max(max_tilt,tilt);peak=max(peak,float(max(abs(d.actuator_force))))
        if fall is None and (tilt>45 or d.xpos[pelvis,2]<.5*initial_z):fall=float(d.time)
        power=d.actuator_force*d.qvel[ndof]
        # Integrate locomotion effort after the one-second standing setup.
        if t>=1:
            positive_work+=float(np.maximum(power,0).sum())*dt
            tau2+=float(np.dot(d.actuator_force,d.actuator_force))*dt
            saturated+=int(any(abs(d.actuator_force[:12])>=1.499))
            active_duration+=dt
        contact=[False,False];force=[0.,0.];speeds=[[],[]]
        for ci in range(d.ncon):
            c=d.contact[ci]
            for j,s in enumerate(soles):
                if {int(c.geom1),int(c.geom2)}=={s,ground}:
                    contact[j]=True
                    f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);force[j]+=float(f[0])
                    if f[0]>.2:
                        velocity=np.zeros(6)
                        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_GEOM,s,velocity,0)
                        pointvelocity=velocity[3:]+np.cross(velocity[:3],c.pos-d.geom_xpos[s])
                        speeds[j].append(float(np.linalg.norm(pointvelocity[:2])))
        if t>=1:slip+=sum(np.mean(v) for v in speeds if v)*dt
        if task10 is None and d.xpos[pelvis,0]-initial_x>=.10 and fall is None:
            task10=dict(time_s=float(d.time),positive_work_J=positive_work,torque_squared_Nm2s=tau2)
        single+=int(contact[0]!=contact[1])
        if record and i%round(.02/dt)==0:
            rows.append(dict(t=round(float(d.time),4),x=float(d.xpos[pelvis,0]),tilt=tilt,torque=float(max(abs(d.actuator_force))),left_N=force[0],right_N=force[1],work_J=positive_work,torque_squared=tau2))
        if record and i%round((1/30)/dt)==0:
            frames.append(d.qpos.copy().tolist())
    distance=float(d.xpos[pelvis,0])-initial_x
    result=dict(design=design,gait=gait,period=period,kp=kp,kv=kv,spring=spring,feedback=feedback,scenario=scenario,push_N=push if scenario.startswith('push') else 0,mu=mu,phone_g=phone*1000,dt=dt,duration_s=duration,mass_kg=float(sum(m.body_mass)),initial_COM_mm=com*1000,height_mm=height*1000,travel_m=distance,max_tilt_deg=max_tilt,fall_s=fall,positive_work_J=positive_work,torque_squared_Nm2s=tau2,work_per_m=positive_work/distance if distance>.02 and fall is None else None,torque_squared_per_m=tau2/distance if distance>.02 and fall is None else None,peak_torque_Nm=peak,saturation_time_s=saturated*dt,single_contact_fraction=single/(duration/dt),contact_slip_path_m=slip,first_10cm=task10)
    if record:
        stem=('v2' if design!='baseline' else 'v1')+'-'+scenario
        (OUT/(stem+'-model.xml')).write_text(xml)
        (OUT/(stem+'-motion.json')).write_text(json.dumps({'qpos':frames,'sample_interval_s':round((1/30)/dt)*dt,'telemetry':rows,'summary':result}))
        with (OUT/(stem+'-effort.csv')).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    return result

def main():
    sweep=[]
    configs=[dict(design='baseline'),dict(design='compact')]
    for period in [.85,.95,1.05]:
        for gait in [.8,1.,1.15]:
            for stiffness in [0.,.2]:
                configs.append(dict(design='compact',gait=gait,period=period,kp=16.,kv=.45,spring=stiffness))
    for cfg in configs:
        r=experiment(**cfg);sweep.append(r)
        print(json.dumps({k:r[k] for k in ['design','gait','period','spring','travel_m','work_per_m','torque_squared_per_m','max_tilt_deg','fall_s']}),flush=True)
    (OUT/'design-sweep.json').write_text(json.dumps(sweep,indent=2))

if __name__=='__main__':main()
