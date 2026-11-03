"""Targeted checks for the live environment and delivered benchmarks."""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_lab import Environment,configuration,run
root=Path(__file__).resolve().parents[1]
for invalid in [{'phone_g':0},{'period':float('nan')},{'terrain':'unknown'},{'dt':0},{'extra':True}]:
    try:configuration(invalid)
    except ValueError:pass
    else:raise AssertionError(invalid)
env=Environment();env.advance(.1);assert env.d.time>0
old=env.d.qvel.copy();env.push(force=4);env.advance(.1);assert any(abs(env.d.qvel-old)>.001)
assert env.reset()['metrics']['time_s']==0
rows=json.loads((root/'speed-sweep.json').read_text());validation=json.loads((root/'speed-validation.json').read_text())
baseline=rows[0]['metrics'];fast=next(r['metrics'] for r in rows if r['config']['design']=='v3' and r['config']['period']==.5 and r['config']['amplitude']==1.5)
assert fast['mean_speed_m_s']>baseline['mean_speed_m_s']*4
assert fast['slip_per_travel']<baseline['slip_per_travel']
assert fast['positive_work_J']>baseline['positive_work_J']
assert all(r['metrics']['fall_s'] is None for r in validation if r['test']=='long_flat')
limited=run(dict(design='v3',period=.5,amplitude=1.5,no_load_speed=4),duration=8)
checks={'passed':True,'observed_fast_speed_ratio':fast['mean_speed_m_s']/baseline['mean_speed_m_s'],'limited_motor_test':limited,'note':'Finite synthetic tests; substantial slip remains. Mechanical effort does not estimate battery runtime.'}
(root/'speed-checks.json').write_text(json.dumps(checks,indent=2));print(json.dumps(checks),flush=True)
