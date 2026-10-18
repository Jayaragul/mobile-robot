"""Render recorded states from improve_design.experiment(record=True)."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import json,subprocess
import numpy as np
import mujoco,imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
OUT=Path(__file__).resolve().parent
def render(case):
    recs=[json.loads((OUT/f'{v}-{case}-motion.json').read_text()) for v in ['v1','v2']]
    models=[mujoco.MjModel.from_xml_path(str(OUT/f'{v}-{case}-model.xml')) for v in ['v1','v2']]
    datas=[mujoco.MjData(m) for m in models]
    renders=[mujoco.Renderer(m,height=460,width=480) for m in models]
    cam=mujoco.MjvCamera();cam.distance=.73;cam.azimuth=135;cam.elevation=-16
    font=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',18)
    title=ImageFont.truetype('C:/Windows/Fonts/segoeuib.ttf',24)
    dt=recs[0]['sample_interval_s']
    writer=subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-f','rawvideo','-s','960x640','-pix_fmt','rgb24','-r',str(1/dt),'-i','-','-an','-c:v','libx264','-threads','1','-crf','20','-pix_fmt','yuv420p',str(OUT/f'revised-{case}-comparison.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    for fi in range(min(len(r['qpos']) for r in recs)):
        t=.002+fi*dt
        canvas=Image.new('RGB',(960,640),(18,29,24));draw=ImageDraw.Draw(canvas)
        for j in range(2):
            m,d=models[j],datas[j]
            d.qpos[:]=recs[j]['qpos'][fi];mujoco.mj_forward(m,d)
            pelvis=m.body('pelvis').id
            cam.lookat[:]=[d.xpos[pelvis,0],d.xpos[pelvis,1],.155]
            renders[j].update_scene(d,cam);canvas.paste(Image.fromarray(renders[j].render()),(j*480,80))
            row=min(recs[j]['telemetry'],key=lambda r:abs(r['t']-t))
            draw.text((20+j*480,12),'V1 / ORIGINAL' if j==0 else 'V2 / COMPACT + FEEDBACK',font=title,fill=(233,240,228))
            draw.text((20+j*480,46),'Portrait phone · 80 mm links' if j==0 else 'Landscape phone · 70 mm links',font=font,fill=(163,195,162))
            draw.text((20+j*480,550),f't {t:.2f} s  |  body tilt {row["tilt"]:.1f}°',font=font,fill=(222,232,215))
            if case=='step':
                draw.text((20+j*480,579),f'Travel {row["x"]*100:.1f} cm · motor work {row["work_J"]:.2f} J',font=font,fill=(163,195,162))
            else:
                fall=recs[j]['summary']['fall_s'];label='FALL DETECTED' if fall and t>=fall else 'UPRIGHT'
                draw.text((20+j*480,579),label+' · 12 N lateral push at t=2 s',font=font,fill=(244,188,145) if label.startswith('FALL') else (163,195,162))
            if 2<=t<2.16 and case=='push_side':draw.text((20+j*480,112),'← 12 N PUSH',font=title,fill=(244,211,143))
        draw.line((480,0,480,640),fill=(88,109,85),width=1)
        draw.text((20,614),'MuJoCo dynamics · assumed hardware · robot–floor contact only · no battery model',font=font,fill=(142,168,140))
        writer.stdin.write(np.asarray(canvas).tobytes())
        if fi==60:canvas.save(OUT/f'revised-{case}-preview.png')
    writer.stdin.close();err=writer.stderr.read();ret=writer.wait()
    for r in renders:r.close()
    if ret:raise RuntimeError(err.decode(errors='replace'))
    print('Rendered',case,flush=True)
if __name__=='__main__':
    render('step');render('push_side')
