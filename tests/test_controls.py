import json,threading,unittest,urllib.request,urllib.error
from unittest.mock import patch
from lab_runtime import Controller,Conflict
from lab_server import make_server
from replay_run import replay
from robot_lab import configuration

CLIENT='operator-test-123'
class Clock:
    def __init__(self):self.t=0.
    def __call__(self):return self.t

class LifecycleTests(unittest.TestCase):
    def setUp(self):self.clock=Clock();self.runtime=Controller(clock=self.clock)
    def tearDown(self):self.runtime.close()
    def advance(self,seconds=.02):self.clock.t+=seconds;self.runtime.advance_wall()
    def test_run_pause_reset(self):
        r=self.runtime;r.command(CLIENT,'start');self.advance();self.assertGreater(r.env.d.time,0)
        with self.assertRaises(Conflict):r.reset(CLIENT,{})
        r.command(CLIENT,'pause');stamp=r.env.d.time;self.advance();self.assertEqual(r.env.d.time,stamp)
        r.command(CLIENT,'step');self.assertAlmostEqual(r.env.d.time,stamp+r.env.c.dt)
        old=r.run_id;r.reset(CLIENT,{});self.assertNotEqual(r.run_id,old);self.assertEqual(r.env.d.time,0)
    def test_stop_requires_ack_and_reset(self):
        r=self.runtime;r.command(CLIENT,'start');r.command(CLIENT,'stop');self.advance()
        for action in ['start','step']:
            with self.assertRaises(Conflict):r.command(CLIENT,action)
        with self.assertRaises(Conflict):r.reset(CLIENT,{})
        r.command(CLIENT,'acknowledge');self.assertEqual(r.state,'NEEDS_RESET')
        with self.assertRaises(Conflict):r.command(CLIENT,'start')
        r.reset(CLIENT,{});r.command(CLIENT,'start');self.assertEqual(r.state,'RUNNING')
    def test_watchdog_latches_without_browser(self):
        r=self.runtime;r.command(CLIENT,'start');self.advance(2.1)
        self.assertEqual(r.state,'FAULT');self.assertEqual(r.reason,'operator_connection_lost')
        r.heartbeat(CLIENT);self.assertEqual(r.state,'FAULT')
    def test_operator_conflict(self):
        self.runtime.command(CLIENT,'start')
        with self.assertRaises(Conflict):self.runtime.command('other-operator','pause')
        self.runtime.command('other-operator','stop');self.assertEqual(self.runtime.state,'STOPPED')
    def test_invalid_reset_preserves_model(self):
        r=self.runtime;old=r.env
        with self.assertRaises(ValueError):r.reset(CLIENT,{'phone_g':0})
        self.assertIs(r.env,old);self.assertEqual(r.state,'READY')
    def test_catchup_is_bounded_and_reported(self):
        r=self.runtime;r.command(CLIENT,'start');self.advance(.5)
        self.assertLessEqual(r.env.d.time,.052);self.assertAlmostEqual(r.lag_discarded_s,.45)
    def test_numerical_failure_latches_and_preserves_valid_state(self):
        r=self.runtime;r.command(CLIENT,'start')
        def broken():r.env.d.qpos[0]=float('nan');raise RuntimeError('test solver failure')
        with patch.object(r.env,'step',side_effect=broken):self.advance()
        self.assertEqual(r.state,'FAULT');self.assertEqual(r.reason,'physics_error')
        json.dumps(r.snapshot(),allow_nan=False)
    def test_replay_includes_disturbances_and_hashes(self):
        r=self.runtime;r.command(CLIENT,'start')
        for _ in range(10):self.advance()
        r.push(CLIENT,'side',4.)
        for _ in range(10):self.advance()
        r.command(CLIENT,'pause');record=r.export()
        self.assertEqual(len(record['software']['source_sha256']),4)
        result=replay(record);self.assertTrue(result['matched'],result)
        record['model_sha256']='bad'
        with self.assertRaises(ValueError):replay(record)
    def test_configuration_rejects_invalid_and_nonfinite_inputs(self):
        for value in [[],{'period':True},{'period':float('nan')},{'period':float('inf')},{'unknown':1}]:
            with self.subTest(value=value),self.assertRaises((ValueError,TypeError)):configuration(value)

class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime=Controller();cls.server=make_server(0,cls.runtime)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
        cls.url='http://127.0.0.1:'+str(cls.server.server_port)
    @classmethod
    def tearDownClass(cls):cls.server.shutdown();cls.server.server_close();cls.runtime.close();cls.thread.join()
    def request(self,path,data=None,origin=None):
        headers={'Content-Type':'application/json'}
        if origin:headers['Origin']=origin
        request=urllib.request.Request(self.url+path,data=None if data is None else json.dumps(data).encode(),headers=headers)
        try:
            with urllib.request.urlopen(request,timeout=3) as response:return response.status,response.read()
        except urllib.error.HTTPError as error:return error.code,error.read()
    def test_health_and_static_client(self):
        self.assertEqual(self.request('/api/health')[0],200);self.assertEqual(self.request('/lab-client.js')[0],200)
    def test_cross_origin_and_unknown_fields_are_rejected(self):
        self.assertEqual(self.request('/api/control',{'client_id':CLIENT,'command':'start'},'https://example.com')[0],403)
        self.assertEqual(self.request('/api/control',{'client_id':CLIENT,'command':'start','unknown':1})[0],400)
    def test_step_legacy_endpoint_is_removed(self):self.assertEqual(self.request('/api/step',{})[0],404)
    def test_export_schema(self):
        status,body=self.request('/api/run');self.assertEqual(status,200);self.assertEqual(json.loads(body)['schema_version'],1)

if __name__=='__main__':unittest.main()
