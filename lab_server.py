"""Local engineering simulator: python lab_server.py --port 8766"""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import argparse,csv,io,json,logging
from lab_runtime import Controller,Conflict

ROOT=Path(__file__).resolve().parent
LOG=logging.getLogger('mobile_robot.lab')

def make_server(port=8765,controller=None):
    runtime=controller or Controller(start_worker=True)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,status,body,kind='application/json'):
            raw=body if isinstance(body,bytes) else json.dumps(body,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
            try:self.wfile.write(raw)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            if self.path in ['/','/live-lab.html']:self.send(200,(ROOT/'live-lab.html').read_bytes(),'text/html; charset=utf-8')
            elif self.path=='/lab-client.js':self.send(200,(ROOT/'lab-client.js').read_bytes(),'text/javascript; charset=utf-8')
            elif self.path=='/api/state':self.send(200,runtime.snapshot())
            elif self.path=='/api/health':self.send(200,{'status':'ok','runtime':runtime.snapshot()['runtime']})
            elif self.path=='/api/run':self.send(200,runtime.export())
            elif self.path=='/api/export':
                run=runtime.export();buf=io.StringIO();rows=run['telemetry']
                if rows:w=csv.DictWriter(buf,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
                self.send(200,buf.getvalue().encode(),'text/csv; charset=utf-8')
            else:self.send(404,{'error':'Not found'})
        def do_POST(self):
            origin=self.headers.get('Origin')
            if origin and origin not in [f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}']:
                self.send(403,{'error':'Local origin required'});return
            try:
                count=int(self.headers.get('Content-Length','0'))
                if not 0<count<=10000:raise ValueError('Invalid request size')
                if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('JSON required')
                data=json.loads(self.rfile.read(count))
                if not isinstance(data,dict):raise ValueError('Object required')
                routes={'/api/reset':{'client_id','config'},'/api/control':{'client_id','command'},'/api/push':{'client_id','axis','force'},'/api/heartbeat':{'client_id'}}
                if self.path not in routes:self.send(404,{'error':'Not found'});return
                if set(data)-routes[self.path]:raise ValueError('Unknown request field')
                client=data.get('client_id')
                if self.path=='/api/reset':
                    if not isinstance(data.get('config'),dict):raise ValueError('config object required')
                    result=runtime.reset(client,data['config'])
                elif self.path=='/api/control':result=runtime.command(client,data.get('command'))
                elif self.path=='/api/push':
                    if isinstance(data.get('force',4.),bool):raise ValueError('force must be numeric')
                    result=runtime.push(client,str(data.get('axis','forward')),float(data.get('force',4.)))
                else:result=runtime.heartbeat(client)
                self.send(200,result)
            except Conflict as error:self.send(409,{'error':str(error)})
            except (ValueError,TypeError) as error:self.send(400,{'error':str(error)})
            except Exception:
                LOG.exception('Request failed');self.send(500,{'error':'Internal error; inspect server logs'})
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.runtime=runtime
    return server

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8765);args=parser.parse_args()
    logging.basicConfig(level=logging.INFO,format='%(asctime)s %(levelname)s %(message)s')
    server=make_server(args.port)
    print(f'Robot simulation: http://127.0.0.1:{server.server_port}',flush=True)
    print('Independent physics loop; browser watchdog enabled. Ctrl+C stops server.',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.runtime.close();server.server_close()
