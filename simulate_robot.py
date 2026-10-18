"""Phone-biped concept: reproducible MuJoCo contact dynamics experiments.
Run: python simulate_robot.py [--render]
Dependencies: mujoco, numpy, pillow, imageio-ffmpeg (rendering only).
All parameters are assumptions, not measured hardware properties.
"""
from pathlib import Path
import argparse, csv, json, math, subprocess, os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
import mujoco

OUT = Path(__file__).resolve().parent
DT = .002

def model_xml(phone_mass=.25, friction=.8):
    legs=[]
    actuators=[]
    for side,y in [('left',.053),('right',-.053)]:
        # Axes: x forward, y left, z up; all lengths in metres.
        legs.append(f'''
        <body name="{side}_yaw" pos="0 {y} -.012">
          <joint name="{side}_hip_yaw" axis="0 0 1" range="-.5 .5"/>
          <geom type="cylinder" size=".017 .006" mass=".025" material="joint"/>
          <body name="{side}_roll">
            <joint name="{side}_hip_roll" axis="1 0 0" range="-.5 .5"/>
            <geom type="sphere" size=".014" mass=".025" material="joint"/>
            <body name="{side}_thigh">
              <joint name="{side}_hip_pitch" axis="0 1 0" range="-1.2 1.2"/>
              <geom type="box" pos="0 0 -.040" size=".012 .013 .040" mass=".100" material="shell"/>
              <body name="{side}_shin" pos="0 0 -.080">
                <joint name="{side}_knee" axis="0 1 0" range="-.05 1.8"/>
                <geom type="box" size=".018 .018 .012" mass=".035" material="joint"/>
                <geom type="box" pos="0 0 -.040" size=".011 .012 .040" mass=".080" material="shell"/>
                <body name="{side}_ankle_pitch" pos="0 0 -.080">
                  <joint name="{side}_ankle_pitch" axis="0 1 0" range="-.9 .9"/>
                  <geom type="sphere" size=".013" mass=".025" material="joint"/>
                  <body name="{side}_foot">
                    <joint name="{side}_ankle_roll" axis="1 0 0" range="-.5 .5"/>
                    <geom name="{side}_sole" type="box" pos="0 0 -.018" size=".048 .030 .006" mass=".080" material="rubber" friction="{friction} .005 .0001"/>
                    <geom type="box" pos="0 0 -.010" size=".043 .027 .002" mass=".015" material="shell"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>''')
        for j in ['hip_yaw','hip_roll','hip_pitch','knee','ankle_pitch','ankle_roll']:
            actuators.append(f'<position name="{side}_{j}" joint="{side}_{j}" kp="22" kv=".65" forcelimited="true" forcerange="-1.5 1.5"/>')
    return f'''<mujoco model="phone_biped_concept">
      <compiler angle="radian" autolimits="true"/>
      <option timestep="{DT}" gravity="0 0 -9.81" integrator="implicitfast" iterations="80"/>
      <visual><global offwidth="960" offheight="640"/><headlight ambient=".4 .4 .4" diffuse=".6 .6 .6"/></visual>
      <default><joint damping=".06" armature=".0003"/><geom contype="2" conaffinity="1" friction="{friction} .005 .0001" condim="3" solref=".008 1" solimp=".95 .99 .001"/></default>
      <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".20 .24 .23" rgb2=".24 .28 .26" width="256" height="256"/>
        <material name="floor" texture="grid" texrepeat="8 8" reflectance=".05"/>
        <material name="shell" rgba=".83 .86 .80 1"/>
        <material name="joint" rgba=".21 .28 .25 1"/>
        <material name="rubber" rgba=".10 .14 .12 1"/>
        <material name="phone" rgba=".12 .16 .15 1"/>
        <material name="screen" rgba=".38 .72 .54 1"/>
      </asset>
      <worldbody>
        <light pos=".4 -.5 1.5" dir="-.2 .2 -1" castshadow="true"/>
        <geom name="ground" type="plane" size="2 2 .1" material="floor" contype="1" conaffinity="2"/>
        <body name="pelvis" pos="0 0 .198">
          <freejoint name="root"/>
          <geom name="battery" type="box" size=".028 .056 .014" mass=".34" material="joint"/>
          <geom name="chassis" type="box" pos="0 0 .021" size=".033 .076 .007" mass=".18" material="shell"/>
          <geom name="electronics" type="box" pos="0 0 .033" size=".021 .035 .005" mass=".06" material="joint"/>
          {''.join(legs)}
          <body name="neck" pos="0 0 .044">
            <joint name="neck_yaw" axis="0 0 1" range="-.65 .65"/>
            <geom type="cylinder" size=".013 .008" mass=".045" material="joint"/>
            <body name="cradle" pos="0 0 .012">
              <joint name="neck_tilt" axis="0 1 0" range="-.35 .35"/>
              <geom type="box" pos="-.008 0 .041" size=".004 .038 .045" mass=".045" material="shell"/>
              <body name="phone" pos="0 0 .041">
                <geom name="phone_body" type="box" size=".0045 .034 .073" mass="{phone_mass}" material="phone"/>
                <geom name="screen" type="box" pos=".0048 0 0" size=".0003 .030 .066" mass=".001" material="screen" contype="0" conaffinity="0"/>
              </body>
            </body>
          </body>
        </body>
      </worldbody>
      <actuator>{''.join(actuators)}
        <position name="neck_yaw" joint="neck_yaw" kp="2" kv=".08" forcelimited="true" forcerange="-.25 .25"/>
        <position name="neck_tilt" joint="neck_tilt" kp="2" kv=".08" forcelimited="true" forcerange="-.25 .25"/>
      </actuator>
    </mujoco>'''

def targets(t, scenario):
    q={}
    for side in ['left','right']:
        for joint in ['hip_yaw','hip_roll','hip_pitch','knee','ankle_pitch','ankle_roll']:
            q[f'{side}_{joint}']=0.
    q.update(neck_yaw=.48*math.sin(max(0,t-1)*1.1) if scenario=='stand' else 0.,neck_tilt=0.)
    if scenario=='step' and t>1:
        # Deliberately basic joint-space gait; no online COM/ZMP feedback.
        ramp=min((t-1)/.6,1.)
        phase=(t-1)*2*math.pi/.95
        for side,offset in [('left',0),('right',math.pi)]:
            p=phase+offset
            lift=max(0,math.sin(p))*ramp
            pitch=.13*math.cos(p)*ramp-.20*lift
            q[side+'_hip_pitch']=pitch
            q[side+'_knee']=.48*lift
            q[side+'_ankle_pitch']=-pitch-.48*lift
            q[side+'_hip_roll']=.065*math.sin(phase)*ramp
            q[side+'_ankle_roll']=-.065*math.sin(phase)*ramp
    return q

def run(scenario,phone_mass=.25,friction=.8,duration=6.,render=False,stem=None):
    xml=model_xml(phone_mass,friction)
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    # Set exact initial sole clearance of 1 mm.
    soleids=[m.geom(x+'_sole').id for x in ['left','right']]
    ground=m.geom('ground').id
    d.qpos[2]+=.007-min(d.geom_xpos[i,2] for i in soleids)
    mujoco.mj_forward(m,d)
    pelvis=m.body('pelvis').id
    names=[m.actuator(i).name for i in range(m.nu)]
    rows=[];falltime=None;max_tilt=0.;peak=0.;lastframe=-1;writer=None;renderer=None
    stem=stem or scenario
    if render:
        from PIL import Image, ImageDraw, ImageFont
        import imageio_ffmpeg
        ff=imageio_ffmpeg.get_ffmpeg_exe()
        writer=subprocess.Popen([ff,'-y','-f','rawvideo','-vcodec','rawvideo','-s','960x640','-pix_fmt','rgb24','-r','30','-i','-','-an','-vcodec','libx264','-crf','21','-pix_fmt','yuv420p',str(OUT/(stem+'.mp4'))],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        renderer=mujoco.Renderer(m,height=640,width=960)
        camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.16];camera.distance=.90;camera.azimuth=135;camera.elevation=-18
        try: font=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',19);titlefont=ImageFont.truetype('C:/Windows/Fonts/segoeuib.ttf',25)
        except OSError: font=titlefont=ImageFont.load_default()
    for i in range(int(duration/DT)):
        t=d.time
        q=targets(t,scenario)
        d.ctrl[:]=[q[n] for n in names]
        d.xfrc_applied[:]=0
        if scenario=='push' and 2.0<=t<2.16:
            # Horizontal push applied at the pelvis COM: 12 N x .16 s = 1.92 N s.
            d.xfrc_applied[pelvis,0]=12.
        mujoco.mj_step(m,d)
        up=d.xmat[pelvis].reshape(3,3)[2,2]
        tilt=math.degrees(math.acos(np.clip(up,-1,1)))
        max_tilt=max(max_tilt,tilt);peak=max(peak,float(np.max(np.abs(d.actuator_force))))
        if falltime is None and (tilt>45 or d.xpos[pelvis,2]<.10):falltime=float(d.time)
        contacts=[0,0];forces=[0.,0.]
        for ci in range(d.ncon):
            c=d.contact[ci]
            for j,g in enumerate(soleids):
                if (c.geom1==g and c.geom2==ground) or (c.geom2==g and c.geom1==ground):
                    force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
                    contacts[j]+=1;forces[j]+=max(0,float(force[0]))
        if i%10==0:
            rows.append(dict(time_s=round(float(d.time),4),x_m=float(d.xpos[pelvis,0]),y_m=float(d.xpos[pelvis,1]),pelvis_z_m=float(d.xpos[pelvis,2]),tilt_deg=tilt,left_contact=bool(contacts[0]),right_contact=bool(contacts[1]),left_normal_N=forces[0],right_normal_N=forces[1],left_sole_centre_z_m=float(d.geom_xpos[soleids[0],2]),right_sole_centre_z_m=float(d.geom_xpos[soleids[1],2]),max_actuator_Nm=float(np.max(np.abs(d.actuator_force)))))
        f=int(d.time*30)
        if render and f!=lastframe:
            renderer.update_scene(d,camera)
            im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,960,102),fill=(18,29,24))
            title={'stand':'01 / STAND + FACE A PERSON','step':'02 / FIRST STEPPING ATTEMPT','push':'03 / PUSH RESPONSE'}[scenario]
            draw.text((24,13),title,font=titlefont,fill=(235,242,233))
            draw.text((24,52),f'MuJoCo dynamics | gravity 9.81 m/s² | phone {phone_mass*1000:.0f} g | friction {friction:.2f}',font=font,fill=(171,199,177))
            draw.rectangle((0,538,960,640),fill=(18,29,24))
            draw.text((24,550),f't = {d.time:4.2f} s     body tilt {tilt:5.1f}°     joint torque peak now {np.max(np.abs(d.actuator_force)):.2f} Nm',font=font,fill=(233,239,230))
            contacttext=('L' if contacts[0] else '—')+' / '+('R' if contacts[1] else '—')
            state='FALL DETECTED' if falltime else 'UPRIGHT'
            draw.text((24,580),f'{state}   |   foot contact {contacttext}   |   model assumptions, not hardware validation',font=font,fill=(225,174,133) if falltime else (171,210,178))
            if scenario=='push' and 2<=t<2.16:draw.text((550,145),'PUSH  12 N →',font=titlefont,fill=(255,210,110))
            writer.stdin.write(np.asarray(im).tobytes())
            if f==30:im.save(OUT/(stem+'-preview.png'))
            lastframe=f
    if renderer:renderer.close()
    if writer:
        writer.stdin.close();err=writer.stderr.read();ret=writer.wait()
        if ret:raise RuntimeError(err.decode(errors='replace'))
    with (OUT/(stem+'-telemetry.csv')).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    single=sum(r['left_contact'] != r['right_contact'] for r in rows)/len(rows)
    return dict(scenario=scenario,phone_mass_g=phone_mass*1000,friction=friction,duration_s=duration,total_mass_kg=float(sum(m.body_mass)),fall_time_s=falltime,max_tilt_deg=max_tilt,peak_actuator_Nm=peak,final_x_m=float(d.xpos[pelvis,0]),final_y_m=float(d.xpos[pelvis,1]),final_pelvis_z_m=float(d.xpos[pelvis,2]),single_foot_contact_fraction=single,timestep_s=DT)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--render',action='store_true');args=parser.parse_args()
    (OUT/'phone-biped-model.xml').write_text(model_xml(),encoding='utf-8')
    results=[]
    for case in ['stand','step','push']:
        result=run(case,render=args.render);results.append(result);print(json.dumps(result),flush=True)
    # Sensitivity cases check the same unmodified controller.
    for mass,mu in [(.15,.8),(.35,.8),(.25,.2)]:
        result=run('step',phone_mass=mass,friction=mu,stem=f'step-{int(mass*1000)}g-mu{mu}');results.append(result);print(json.dumps(result),flush=True)
    (OUT/'physics-results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')

if __name__=='__main__':main()
