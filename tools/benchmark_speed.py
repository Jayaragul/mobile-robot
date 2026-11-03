import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_lab import run
root=Path(__file__).resolve().parents[1]
rows=[]
for design in ['v2','v3']:
    for period in [.95,.75,.6,.5]:
        for amplitude in [1.,1.25,1.5]:
            r=run(dict(design=design,period=period,amplitude=amplitude));rows.append(r)
            m=r['metrics'];print(design,period,amplitude,round(m['mean_speed_m_s'],3),round(m['max_tilt_deg'],2),round(m['slip_per_travel'] or 0,2),round(m['work_J_m'] or 0,1),m['fall_s'],flush=True)
(root/'speed-sweep.json').write_text(json.dumps(rows,indent=2))
