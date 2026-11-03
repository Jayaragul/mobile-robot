"""Server-owned simulator lifecycle; stops here are not hardware safety functions."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
from collections import deque
from dataclasses import asdict
from datetime import datetime,timezone
from pathlib import Path
import hashlib,platform,re,threading,time,uuid
import mujoco,numpy as np
from robot_lab import Environment

VERSION='0.2.0'
class Conflict(ValueError):pass

class Controller:
    def __init__(self,clock=time.monotonic,watchdog_s=2.,start_worker=False):
        self.lock=threading.RLock();self.clock=clock;self.watchdog_s=watchdog_s
        self.env=Environment();self.state='READY';self.reason=None;self.owner=None
        self.heartbeat_at=clock();self.last_tick=clock();self.accumulator=0.
        self.events=deque(maxlen=256);self.inputs=[];self.stop_event=threading.Event();self.worker=None
        self.wall_running_s=0.;self.lag_discarded_s=0.;self.max_batch_s=0.;self.worker_error=None
        self.instance_id=str(uuid.uuid4());self.sequence=0
        self.run_id=str(uuid.uuid4());self.created_at=datetime.now(timezone.utc).isoformat();self._event('initialized')
        if start_worker:self.start_worker()
    def _event(self,event,**details):
        self.events.append(dict(event=event,sim_time_s=float(self.env.d.time),utc=datetime.now(timezone.utc).isoformat(),**details))
    def _operator(self,client):
        if not isinstance(client,str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',client):raise ValueError('Valid client_id required')
        now=self.clock()
        if self.owner not in [None,client] and (self.state=='RUNNING' or now-self.heartbeat_at<=self.watchdog_s):raise Conflict('Another operator owns this session')
        self.owner=client;self.heartbeat_at=now
    def heartbeat(self,client):
        with self.lock:
            if client!=self.owner:raise Conflict('This tab does not own the session; claim an idle session with a preset/reset')
            self.heartbeat_at=self.clock();return self.snapshot()
    def reset(self,client,config):
        with self.lock:
            if self.state=='RUNNING':raise Conflict('Pause before changing configuration')
            if self.state in ['STOPPED','FAULT']:raise Conflict('Acknowledge the latched stop/fault before resetting')
            self._operator(client);candidate=Environment(config)
            self.env=candidate;self.state='READY';self.reason=None;self.inputs=[];self.events.clear()
            self.run_id=str(uuid.uuid4());self.created_at=datetime.now(timezone.utc).isoformat()
            self.wall_running_s=0.;self.lag_discarded_s=0.;self.max_batch_s=0.;self.worker_error=None
            self.accumulator=0.;self.last_tick=self.clock();self._event('reset',config=asdict(candidate.c));return self.snapshot()
    def command(self,client,command):
        with self.lock:
            if command not in ['start','pause','stop','acknowledge','step']:raise ValueError('Unknown control command')
            if command=='stop':
                if not isinstance(client,str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',client):raise ValueError('Valid client_id required')
                self.state='STOPPED';self.reason='operator_stop';self._event('stop_latched');return self.snapshot()
            self._operator(client)
            if command=='start':
                if self.state not in ['READY','PAUSED']:raise Conflict('Start requires ready or paused state')
                self.state='RUNNING';self.last_tick=self.clock();self.accumulator=0.;self._event('started')
            elif command=='pause':
                if self.state not in ['RUNNING','PAUSED']:raise Conflict('Pause requires running or paused state')
                self.state='PAUSED';self._event('paused')
            elif command=='acknowledge':
                if self.state not in ['STOPPED','FAULT']:raise Conflict('No latched stop or fault')
                self.state='NEEDS_RESET';self._event('acknowledged',reason=self.reason)
            elif command=='step':
                if self.state not in ['READY','PAUSED']:raise Conflict('Single step requires ready or paused state')
                try:self._step_physics();self._inspect()
                except Exception as error:self._fault('physics_error',error)
                if self.state in ['READY','PAUSED']:self.state='PAUSED'
                self._event('single_step')
            return self.snapshot()
    def push(self,client,axis,force):
        with self.lock:
            self._operator(client)
            if self.state!='RUNNING':raise Conflict('Apply disturbances only while running')
            self.env.push(axis,force);self.inputs.append(dict(step=self.env.steps,axis=axis,force_N=force,duration_s=.16))
            self._event('push',axis=axis,force_N=force);return self.snapshot()
    def _fault(self,reason,error=None):
        self.state='FAULT';self.reason=reason
        if error is not None:self.worker_error=type(error).__name__
        self._event('fault_latched',reason=reason,error_type=self.worker_error)
    def _step_physics(self):
        d=self.env.d;position=d.qpos.copy();velocity=d.qvel.copy();stamp=float(d.time);steps=self.env.steps
        try:self.env.step()
        except Exception:
            d.qpos[:]=position;d.qvel[:]=velocity;d.time=stamp;self.env.steps=steps
            mujoco.mj_forward(self.env.m,d)
            raise
    def _inspect(self):
        if self.env.fall is not None:self._fault('fall_detected')
        elif self.env.d.time>=30:self.state='COMPLETED';self.reason='duration_limit';self._event('completed')
    def advance_wall(self,now=None):
        with self.lock:
            now=self.clock() if now is None else now;elapsed=max(0.,now-self.last_tick);self.last_tick=now
            if self.state!='RUNNING':return
            if now-self.heartbeat_at>self.watchdog_s:self._fault('operator_connection_lost');return
            self.wall_running_s+=elapsed;allowed=min(elapsed,.05)
            self.lag_discarded_s+=max(0.,elapsed-allowed);self.accumulator+=allowed;start=time.perf_counter()
            try:
                while self.accumulator+1e-12>=self.env.c.dt and self.state=='RUNNING':
                    self._step_physics();self.accumulator-=self.env.c.dt;self._inspect()
            except Exception as error:self._fault('physics_error',error)
            self.max_batch_s=max(self.max_batch_s,time.perf_counter()-start)
    def start_worker(self):
        if self.worker is not None:return
        def loop():
            while not self.stop_event.wait(.005):
                try:self.advance_wall()
                except Exception as error:
                    with self.lock:self._fault('runtime_error',error)
        self.worker=threading.Thread(target=loop,name='mujoco-simulation',daemon=True);self.worker.start()
    def close(self):
        self.stop_event.set()
        if self.worker:self.worker.join(timeout=3.)
    def snapshot(self):
        with self.lock:
            self.sequence+=1
            out=self.env.snapshot();out['runtime']=dict(version=VERSION,instance_id=self.instance_id,sequence=self.sequence,state=self.state,reason=self.reason,run_id=self.run_id,operator_assigned=self.owner is not None,watchdog_s=self.watchdog_s,heartbeat_age_s=max(0.,self.clock()-self.heartbeat_at),wall_running_s=self.wall_running_s,lag_discarded_s=self.lag_discarded_s,max_batch_s=self.max_batch_s,realtime_factor=float(self.env.d.time)/self.wall_running_s if self.wall_running_s else 0.,events=list(self.events)[-12:]);return out
    def export(self):
        with self.lock:
            root=Path(__file__).resolve().parent
            hashes={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in ['robot_lab.py','improve_design.py','simulate_robot.py','lab_runtime.py']}
            return dict(schema_version=1,run_id=self.run_id,created_at=self.created_at,software=dict(lab=VERSION,python=platform.python_version(),mujoco=mujoco.__version__,numpy=np.__version__,source_sha256=hashes),model_sha256=hashlib.sha256(self.env.xml.encode()).hexdigest(),config=asdict(self.env.c),runtime=self.snapshot()['runtime'],summary=self.env.metrics(),events=list(self.events),inputs=list(self.inputs),physics_steps=self.env.steps,final_qpos=self.env.d.qpos.tolist(),final_qvel=self.env.d.qvel.tolist(),telemetry=list(self.env.telemetry),limitations=['assumed hardware','self-collision disabled','ideal sensing','not a battery model','simulation stop only; no hardware safety function'])
