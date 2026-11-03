// UI is a client of the simulator. It never advances physics on a render timer.
const clientId=sessionStorage.getItem('robot-operator')||crypto.randomUUID();
sessionStorage.setItem('robot-operator',clientId);
let owned=false,commandBusy=false,lastRunId=null,networkError=null;
const byId=id=>document.getElementById(id);
function update(s){
 if(state?.runtime?.instance_id===s.runtime.instance_id && state.runtime.sequence>=s.runtime.sequence)return;
 state=s;connected=true;networkError=null;const m=s.metrics,r=s.runtime;
 if(lastRunId!==r.run_id){ids.forEach(id=>{byId(id).value=s.config[id]});labels();lastRunId=r.run_id}
 running=r.state==='RUNNING';byId('time').textContent=m.time_s.toFixed(2)+' s';
 byId('active').textContent=s.config.design.toUpperCase()+' · '+s.config.terrain+' · '+s.config.phone_g+' g phone';
 byId('speed').textContent=(m.mean_speed_m_s*100).toFixed(1)+' cm/s';byId('distance').textContent=(m.travel_m*100).toFixed(1)+' cm';
 byId('tilt').textContent=m.tilt_deg.toFixed(1)+'°';byId('work').textContent=m.positive_work_J.toFixed(2)+' J';
 byId('peak').textContent=m.peak_torque_Nm.toFixed(2)+' Nm';byId('slip').textContent=(m.slip_path_m*100).toFixed(1)+' cm';
 byId('contact').textContent=m.left_N.toFixed(1)+' / '+m.right_N.toFixed(1)+' N';byId('support').textContent=(m.single_support_fraction*100).toFixed(0)+'%';
 byId('phase').textContent=r.state.replaceAll('_',' ');byId('phase').classList.toggle('bad',['FAULT','STOPPED'].includes(r.state));
 byId('play').textContent=running?'Pause':'Start';
 byId('play').disabled=!['READY','PAUSED','RUNNING'].includes(r.state)||commandBusy;
 byId('one').disabled=!['READY','PAUSED'].includes(r.state)||commandBusy;
 ['push','side'].forEach(id=>byId(id).disabled=!running||commandBusy);
 byId('ack').disabled=!['FAULT','STOPPED'].includes(r.state)||commandBusy;
 ['apply','preset-v2','preset-v3','preset-fast'].forEach(id=>byId(id).disabled=['RUNNING','FAULT','STOPPED'].includes(r.state)||commandBusy);
 byId('status').textContent=r.state==='NEEDS_RESET'?'Acknowledged. Apply a preset or reset before restarting.':r.reason?'State: '+r.state+' · '+r.reason.replaceAll('_',' ')+'. Acknowledge latched faults before resetting.':'Connected · controller '+r.version+' · server-owned physics · '+(owned?'this tab is the operator':'viewing; Start or Reset claims an available session');
 byId('timing').textContent='Real-time factor '+r.realtime_factor.toFixed(2)+'× · maximum physics batch '+(r.max_batch_s*1000).toFixed(1)+' ms · discarded wall-time backlog '+r.lag_discarded_s.toFixed(3)+' s';
 byId('run-id').textContent='Run '+r.run_id.slice(0,8)+' · integration '+(s.config.dt*1000).toFixed(1)+' ms · watchdog '+r.watchdog_s.toFixed(1)+' s';
 byId('events').replaceChildren(...r.events.slice().reverse().map(e=>{const el=document.createElement('li');el.textContent=e.sim_time_s.toFixed(3)+' s  '+e.event.replaceAll('_',' ')+(e.reason?' · '+e.reason.replaceAll('_',' '):'');return el}));
 draw();
}
function failure(error){networkError=error.message;byId('status').textContent=error.message;}
async function send(path,body){
 if(commandBusy && body.command!=='stop')return;commandBusy=true;
 try{const s=await api(path,{...body,client_id:clientId});if(body.command!=='stop')owned=true;commandBusy=false;update(s)}
 catch(error){commandBusy=false;failure(error)}
}
async function poll(){
 try{if(!commandBusy){const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error('Simulator unavailable');update(await r.json())}}
 catch(error){connected=false;owned=false;['play','one','push','side','apply'].forEach(id=>byId(id).disabled=true);byId('phase').textContent='DISCONNECTED';failure(Error('Connection lost. The server watchdog latches a fault after 2 seconds. Reconnect, acknowledge, then reset.'))}
 finally{setTimeout(poll,100)}
}
async function heartbeat(){
 if(owned){try{await api('/api/heartbeat',{client_id:clientId})}catch(error){owned=false;failure(error)}}
 setTimeout(heartbeat,300);
}
byId('play').onclick=()=>send('/api/control',{command:running?'pause':'start'});
byId('one').onclick=()=>send('/api/control',{command:'step'});
byId('stop').onclick=()=>send('/api/control',{command:'stop'});
byId('ack').onclick=()=>send('/api/control',{command:'acknowledge'});
byId('apply').onclick=()=>send('/api/reset',{config:values()});
byId('push').onclick=()=>send('/api/push',{axis:'forward',force:4});byId('side').onclick=()=>send('/api/push',{axis:'side',force:4});
function preset(v,fast=false){
 byId('design').value=v;byId('period').value=fast?.5:.95;byId('amplitude').value=v==='v3'?1.5:1;
 byId('terrain').value='flat';byId('phone_g').value=250;byId('friction').value=.8;byId('torque').value=1.5;byId('no_load_speed').value=8;labels();send('/api/reset',{config:values()});
}
byId('preset-v2').onclick=()=>preset('v2');byId('preset-v3').onclick=()=>preset('v3');byId('preset-fast').onclick=()=>preset('v3',true);
byId('save').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify(state?.config||values(),null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='robot-config.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};
labels();draw();poll();heartbeat();
