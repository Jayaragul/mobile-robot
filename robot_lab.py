"""Local MuJoCo robot environment: parameterized design, gait and floor tests.

V3 is an experimental speed candidate, not a validated walking robot.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import math, threading, xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict
import numpy as np
import mujoco
from improve_design import design_xml

@dataclass
class Config:
    design:str='v2'
    period:float=.95
    amplitude:float=1.
    phone_g:float=250.
    friction:float=.8
    terrain:str='flat'
    torque:float=1.5
    no_load_speed:float=8.
    feedback:float=.5
    dt:float=.002

def configuration(values=None):
    if values is None:values={}
    if not isinstance(values,dict):raise ValueError('Configuration must be an object')
    allowed=set(Config.__dataclass_fields__)
    if set(values)-allowed:raise ValueError('Unknown configuration field')
    c=Config(**values)
    if c.design not in ['v2','v3']:raise ValueError('Unknown design')
    if c.terrain not in ['flat','ramp','obstacle']:raise ValueError('Unknown terrain')
    ranges={'period':(.4,1.4),'amplitude':(.4,1.8),'phone_g':(150,350),'friction':(.15,1.2),'torque':(.5,2.),'no_load_speed':(4,16),'feedback':(0,1.5),'dt':(.001,.004)}
    for k,(lo,hi) in ranges.items():
        if isinstance(getattr(c,k),bool):raise ValueError(f'{k} must be numeric')
        val=float(getattr(c,k))
        if not math.isfinite(val) or not lo<=val<=hi:raise ValueError(f'{k} must be between {lo} and {hi}')
        setattr(c,k,val)
    return c

def xml_for(c):
    root=ET.fromstring(design_xml('compact',c.phone_g/1000,c.friction,c.dt))
    if c.design=='v3':
        # Keep low landscape phone; lengthen links without increasing their target mass.
        # These masses are design targets requiring structural validation.
        for body in root.findall('.//body'):
            if body.get('name','').endswith(('_shin','_ankle_pitch')):body.set('pos','0 0 -.080')
        for g in root.findall('.//geom'):
            if g.get('pos')=='0 0 -.035':
                g.set('pos','0 0 -.040');dims=g.get('size').split();dims[2]='.040';g.set('size',' '.join(dims))
        for side in ['left','right']:
            sole=root.find(f'.//geom[@name="{side}_sole"]')
            sole.set('size','.055 .034 .006');sole.set('mass','.075')
            for g in root.find(f'.//body[@name="{side}_foot"]').findall('geom'):
                if g is not sole:g.set('size','.050 .031 .002');g.set('mass','.016')
    if c.terrain=='ramp':
        # Ramp begins beyond initial stance: 4 mm rise over 200 mm (1.15 degrees).
        ET.SubElement(root.find('worldbody'),'geom',name='ramp',type='box',pos='.35 0 .001',size='.10 .30 .002',euler='0 -.02 0',rgba='.61 .55 .36 1',contype='1',conaffinity='2',friction=f'{c.friction} .005 .0001')
    elif c.terrain=='obstacle':
        ET.SubElement(root.find('worldbody'),'geom',name='obstacle',type='box',pos='.27 0 .0025',size='.018 .30 .0025',rgba='.61 .55 .36 1',contype='1',conaffinity='2',friction=f'{c.friction} .005 .0001')
    return ET.tostring(root,encoding='unicode')

class Environment:
    def __init__(self,values=None):
        self.lock=threading.RLock();self.reset(values)
    def reset(self,values=None):
        with self.lock:
            self.c=configuration(values);self.xml=xml_for(self.c)
            self.m=mujoco.MjModel.from_xml_string(self.xml);self.d=mujoco.MjData(self.m)
            mujoco.mj_forward(self.m,self.d)
            self.pelvis=self.m.body('pelvis').id
            self.soles=[self.m.geom(s+'_sole').id for s in ['left','right']]
            self.d.qpos[2]+=.007-min(self.d.geom_xpos[s,2] for s in self.soles)
            mujoco.mj_forward(self.m,self.d)
            self.initial_z=float(self.d.xpos[self.pelvis,2]);self.x0=float(self.d.xpos[self.pelvis,0])
            self.names=[self.m.actuator(i).name for i in range(self.m.nu)]
            self.dofs=[self.m.jnt_dofadr[self.m.actuator_trnid[i,0]] for i in range(self.m.nu)]
            self.work=0.;self.tau2=0.;self.slip=0.;self.fall=None;self.max_tilt=0.;self.peak_torque=0.;self.peak_speed=0.;self.single_steps=0;self.active_steps=0
            self.sat_steps=0;self.steps=0;self.push_until=0.;self.push_axis=0;self.push_force=0.;self.telemetry=[];self.contact=[False,False];self.loads=[0.,0.];self.bound_hit=False
            self.mass=float(sum(self.m.body_mass))
            return self.snapshot()
    def push(self,axis='forward',force=4.,duration=.16):
        if axis not in ['forward','side']:raise ValueError('Invalid push direction')
        if not math.isfinite(force) or not 0<=force<=12:raise ValueError('Push must be 0–12 N')
        with self.lock:self.push_axis=0 if axis=='forward' else 1;self.push_force=force;self.push_until=float(self.d.time)+duration
    def targets(self):
        c,d=self.c,self.d;t=float(d.time)
        q={n:0. for n in self.names}
        if t>1:
            ramp=min((t-1)/.6,1.);phase=(t-1)*2*math.pi/c.period
            for side,offset in [('left',0),('right',math.pi)]:
                lift=max(0,math.sin(phase+offset))*ramp*c.amplitude
                pitch=.13*math.cos(phase+offset)*ramp*c.amplitude-.20*lift
                q[side+'_hip_pitch']=pitch;q[side+'_knee']=.48*lift;q[side+'_ankle_pitch']=-pitch-.48*lift
                q[side+'_hip_roll']=.065*math.sin(phase)*ramp*c.amplitude;q[side+'_ankle_roll']=-q[side+'_hip_roll']
        R=d.xmat[self.pelvis].reshape(3,3)
        pitch=math.atan2(R[0,2],R[2,2]);roll=math.atan2(-R[1,2],R[2,2])
        for side in ['left','right']:
            q[side+'_ankle_pitch']+=float(np.clip(c.feedback*(pitch+.07*d.cvel[self.pelvis,1]),-.25,.25))
            q[side+'_ankle_roll']+=float(np.clip(c.feedback*(roll+.07*d.cvel[self.pelvis,0]),-.20,.20))
        return [q[n] for n in self.names]
    def advance(self,seconds=1/30):
        with self.lock:
            for _ in range(max(1,round(seconds/self.c.dt))):
                if self.fall is not None or self.d.time>=30:break
                self.step()
            return self.snapshot()
    def step(self):
        m,d,c=self.m,self.d,self.c
        d.ctrl[:]=self.targets()
        # Assumed joint-output torque/speed envelope, not a measured motor curve.
        limits=c.torque*np.maximum(0.,1-np.abs(d.qvel[self.dofs[:12]])/c.no_load_speed)
        m.actuator_forcerange[:12,0]=-limits;m.actuator_forcerange[:12,1]=limits
        d.xfrc_applied[:]=0
        if d.time<self.push_until:d.xfrc_applied[self.pelvis,self.push_axis]=self.push_force
        mujoco.mj_step(m,d);self.steps+=1
        tilt=math.degrees(math.acos(float(np.clip(d.xmat[self.pelvis].reshape(3,3)[2,2],-1,1))))
        self.max_tilt=max(self.max_tilt,tilt)
        speed=float(max(abs(d.qvel[self.dofs[:12]])));self.peak_speed=max(self.peak_speed,speed)
        self.peak_torque=max(self.peak_torque,float(max(abs(d.actuator_force))))
        if tilt>45 or d.xpos[self.pelvis,2]<self.initial_z*.5:self.fall=float(d.time)
        if not np.all(np.isfinite(d.qpos)):raise RuntimeError('Non-finite physics state')
        active=d.time>=1.
        if active:
            self.active_steps+=1
            self.work+=float(np.maximum(d.actuator_force*d.qvel[self.dofs],0).sum())*c.dt
            self.tau2+=float(np.dot(d.actuator_force,d.actuator_force))*c.dt
            self.sat_steps+=int(any(abs(d.actuator_force[:12])>=np.maximum(limits-.001,.001)))
        self.contact=[False,False];self.loads=[0.,0.];speeds=[[],[]]
        for ci in range(d.ncon):
            contact=d.contact[ci]
            for j,sole in enumerate(self.soles):
                other=contact.geom2 if contact.geom1==sole else contact.geom1 if contact.geom2==sole else None
                if other is not None and m.geom_bodyid[other]==0:
                    f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
                    if f[0]>.2:
                        self.contact[j]=True;self.loads[j]+=float(f[0])
                        v=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_GEOM,sole,v,0)
                        vp=v[3:]+np.cross(v[:3],contact.pos-d.geom_xpos[sole]);speeds[j].append(float(np.linalg.norm(vp[:2])))
        if active:
            self.slip+=sum(np.mean(v) for v in speeds if v)*c.dt
            self.single_steps+=int(self.contact[0]!=self.contact[1])
        if self.steps%max(1,round(.02/c.dt))==0:self.telemetry.append(self.metrics())
    def metrics(self):
        t=float(self.d.time);x=float(self.d.xpos[self.pelvis,0])-self.x0
        return dict(time_s=t,travel_m=x,mean_speed_m_s=x/max(t-1,.001) if t>1 else 0.,tilt_deg=math.degrees(math.acos(float(np.clip(self.d.xmat[self.pelvis].reshape(3,3)[2,2],-1,1)))),max_tilt_deg=self.max_tilt,positive_work_J=self.work,work_J_m=self.work/x if x>.02 else None,torque_squared_Nm2s=self.tau2,slip_path_m=self.slip,slip_per_travel=self.slip/x if x>.02 else None,single_support_fraction=self.single_steps/max(1,self.active_steps),peak_joint_speed_rad_s=self.peak_speed,peak_torque_Nm=self.peak_torque,saturation_fraction=self.sat_steps/max(1,self.active_steps),fall_s=self.fall,left_N=self.loads[0],right_N=self.loads[1],mass_kg=self.mass)
    def snapshot(self):
        geoms=[]
        for i in range(self.m.ngeom):
            if self.m.geom_type[i]==mujoco.mjtGeom.mjGEOM_PLANE:continue
            geoms.append(dict(type=int(self.m.geom_type[i]),size=self.m.geom_size[i].tolist(),pos=self.d.geom_xpos[i].tolist(),rot=self.d.geom_xmat[i].tolist(),color=self.m.mat_rgba[self.m.geom_matid[i]].tolist() if self.m.geom_matid[i]>=0 else self.m.geom_rgba[i].tolist()))
        return dict(config=asdict(self.c),metrics=self.metrics(),geoms=geoms)

def run(values,duration=8.):
    env=Environment(values)
    while env.d.time<duration and env.fall is None:env.advance(min(.1,duration-env.d.time))
    return dict(config=asdict(env.c),metrics=env.metrics())
