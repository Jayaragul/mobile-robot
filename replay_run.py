"""Replay a downloaded /api/run export against the installed simulator source."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from robot_lab import Environment

def replay(record):
    if record.get('schema_version')!=1:raise ValueError('Unsupported run schema')
    env=Environment(record['config'])
    if hashlib.sha256(env.xml.encode()).hexdigest()!=record['model_sha256']:raise ValueError('Model differs from recorded run')
    count=record['physics_steps']
    if isinstance(count,bool) or not isinstance(count,int) or not 0<=count<=round(31/env.c.dt):raise ValueError('Invalid physics step count')
    inputs=record.get('inputs',[]);at=0
    for step in range(count):
        while at<len(inputs) and inputs[at]['step']==step:
            event=inputs[at];env.push(event['axis'],event['force_N'],event['duration_s']);at+=1
        env.step()
    qerror=float(np.max(np.abs(env.d.qpos-np.asarray(record['final_qpos']))));verror=float(np.max(np.abs(env.d.qvel-np.asarray(record['final_qvel']))))
    return dict(physics_steps=count,max_qpos_error=qerror,max_qvel_error=verror,matched=qerror<1e-7 and verror<1e-7)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_json');args=p.parse_args();result=replay(json.loads(Path(args.run_json).read_text()))
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['matched'] else 1)
