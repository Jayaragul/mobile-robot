import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_lab import run
root=Path(__file__).resolve().parents[1]
rows=[]
for name,preset in [('v2',dict(design='v2')),('v3_conservative',dict(design='v3',amplitude=1.5,period=.95)),('v3_fast',dict(design='v3',amplitude=1.5,period=.5))]:
    for test,values,duration in [('long_flat',{},20),('fine_timestep',{'dt':.001},8),('heavy_phone',{'phone_g':350},12),('lower_grip',{'friction':.4},12),('ramp',{'terrain':'ramp'},12),('obstacle',{'terrain':'obstacle'},12)]:
        r=run(dict(**preset,**values),duration);r['preset']=name;r['test']=test;rows.append(r)
        m=r['metrics'];print(name,test,round(m['mean_speed_m_s'],3),round(m['max_tilt_deg'],1),m['fall_s'],flush=True)
(root/'speed-validation.json').write_text(json.dumps(rows,indent=2))
