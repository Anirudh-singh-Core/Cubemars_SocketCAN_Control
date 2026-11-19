"""Offline GUI integration tests. No physical CAN access or motor activation."""
import json
from pathlib import Path
import re
import sys
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'can_controller'/'tests'))
from test_core import ScriptedTransport,ready,wait
from handycan import Session,can_test
from handycan.protocol import ENABLE,DISABLE,SET_ZERO
from motor_controller_beta_v2.app import Application
from motor_controller_beta_v2.web import make_server

CONFIG=dict(channel='can0',control_hz=100,motors=[dict(motor_id=1,reply_id=1,
    joint_min=0,joint_max=360,speed_max=280,torque_max=5,kp_max=50,kd_max=5)])

@pytest.fixture
def app(tmp_path,monkeypatch):
    monkeypatch.setattr('motor_controller_beta_v2.app.ROOT',tmp_path)
    def factory(*args,**kw):
        return Session(*args,**kw,_transport_factory=ScriptedTransport,_inspector=lambda _:None)
    a=Application(factory,lambda s,i,**kw:can_test(s,i,**kw,window=.08,_inspector=ready))
    yield a
    a.close()

def connect(app):
    app.action('configure',{'config':CONFIG});app.action('connect',{})
    return app.session

def test_startup_and_connect_never_transmit(app):
    assert not app.snapshot()['connected'] and app.session is None
    with pytest.raises(RuntimeError):app.action('connect',{})
    session=connect(app)
    assert session._transport.sent==[]
    app.action('disconnect',{})
    assert session._transport is None and app.session is None


def test_timing_policy_roundtrip_and_validation(app):
    config=dict(CONFIG,timing={'tx_confirmation_timeout_s':.025,'special_response_window_s':.02})
    app.action('configure',{'config':config})
    assert app.snapshot()['config']['timing']['tx_confirmation_timeout_s']==.025
    app.action('connect',{})
    assert app.session.timing.special_response_window_s==.02
    assert app.session.snapshot()['timing']['tx_confirmation_timeout_s']==.025
    assert app.session._transport.sent==[]
    app.action('disconnect',{})
    with pytest.raises(ValueError):
        app.action('configure',{'config':dict(CONFIG,timing={'startup_feedback_timeout_s':-1})})

def test_confirmed_activation_zero_reenable_and_stop(app):
    s=connect(app);s._transport.reply=True
    with pytest.raises(ValueError):app.action('enable',{'motor_id':1})
    assert s._transport.sent==[]
    app.action('enable',{'motor_id':1,'confirm':True})
    wait(s,lambda x:x['axes'][1]['state']=='MONITORING')
    assert app.snapshot()['readings'][1]['valid']
    app.action('disable',{'motor_id':1})
    app.action('zero',{'motor_id':1,'confirm':True})
    app.action('enable',{'motor_id':1,'confirm':True})
    wait(s,lambda x:x['axes'][1]['state']=='MONITORING')
    # Stop bypasses the HTTP action mutex.
    app._actions.acquire()
    try:app.stop()
    finally:app._actions.release()
    wait(s,lambda x:x['axes'][1]['state']=='DISABLED')
    assert SET_ZERO in [p for _,p,_ in s._transport.sent]
    assert app.snapshot()['initialized']

def test_can_test_reuses_session_and_saves_report(app):
    s=connect(app);t=s._transport;t.reply=True
    with pytest.raises(ValueError):app.action('test',{'motor_id':1})
    app.action('test',{'motor_id':1,'confirm':True})
    app._job_thread.join(3)
    report=app.snapshot()['job']['report']
    assert report['result']=='PASS',report
    assert s._transport is t and s.snapshot()['axes'][1]['state']=='DISABLED'
    assert json.loads(s.log_path.with_suffix('.can-test.json').read_text())['result']=='PASS'
    assert len(t.receive_threads)==1

def test_motion_requires_confirmation_and_raw_feedback(app):
    s=connect(app)
    with pytest.raises(ValueError):app.action('move',dict(motor_id=1,target_deg=1,duration=1,kp=1,kd=1))
    assert s._transport.sent==[]
    with pytest.raises(Exception):app.action('move',dict(motor_id=1,target_deg=1,duration=1,kp=1,kd=1,confirm=True))
    assert s._transport.sent==[]

def test_http_get_is_readonly_and_post_requires_token(app):
    server=make_server(app,port=0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    url=f'http://127.0.0.1:{server.server_port}'
    try:
        html=urlopen(url).read().decode();assert 'beta V2' in html
        assert not json.load(urlopen(url+'/api/status'))['connected']
        assert 'drawMotorGraph' in urlopen(url+'/graph.js').read().decode()
        token=re.search("const TOKEN='([^']+)'",html)[1]
        data=json.dumps(dict(action='configure',config=CONFIG)).encode()
        with pytest.raises(HTTPError) as exc:
            urlopen(Request(url+'/api/action',data,{'Content-Type':'application/json'}))
        assert exc.value.code==403
        headers={'Content-Type':'application/json','X-Control-Token':token}
        assert json.load(urlopen(Request(url+'/api/action',data,headers)))['result']['initialized']
        with pytest.raises(HTTPError) as exc:
            urlopen(Request(url+'/api/stop',b'{}',dict(headers,Origin='http://untrusted.example')))
        assert exc.value.code==403
        assert app.session is None
    finally:server.shutdown();server.server_close();thread.join()


def test_gui_test_uses_all_configured_motors_ignoring_selection(app):
    from copy import deepcopy
    config=deepcopy(CONFIG)
    config['motors']=[dict(config['motors'][0],motor_id=i,reply_id=i,
        model='AK70-10-V2' if i==1 else 'AK80-6-V2') for i in (1,2,3)]
    app.action('configure',{'config':config});app.action('connect',{})
    app.session._transport.reply=True
    app.action('test',{'motor_id':1,'confirm':True})
    app._job_thread.join(5)
    job=app.snapshot()['job']
    assert job['state']=='PASS',job
    assert job['motor_ids']==[1,2,3]
    assert job['report']['motor_count']==3
    assert any(p['name']=='combined/polling' for p in job['report']['phases'])


@pytest.mark.parametrize('prime_first',[False,True])
@pytest.mark.parametrize('order',[(1,2,3),(3,2,1)])
def test_multi_motor_configuration_never_requires_priming(app,prime_first,order):
    from copy import deepcopy
    previous=None
    if prime_first:
        previous=connect(app);old_transport=previous._transport;old_worker=previous._worker
        old_transport.reply=True
        app.action('enable',{'motor_id':1,'confirm':True})
        wait(previous,lambda x:x['axes'][1]['state']=='MONITORING')
        app.action('disable',{'motor_id':1})
        app.action('disconnect',{})
        assert old_transport.closed and not old_worker.is_alive()
        assert previous._transport is None and not previous._health.is_alive()
    config=deepcopy(CONFIG)
    config['motors']=[dict(config['motors'][0],motor_id=i,reply_id=i,
        model='AK70-10-V2' if i==1 else 'AK80-6-V2') for i in order]
    app.action('configure',{'config':config});app.action('connect',{})
    s=app.session;t=s._transport
    assert s is not previous and set(s.axes)=={1,2,3}
    assert t.sent==[]  # Configure/connect never primes any motor.
    assert all(a['state']=='DISABLED' and a['feedback'] is None for a in s.axes.values())
    t.reply=True
    # Deliberately start motor 3 first. There is no dependency on motor 1.
    app.action('enable',{'motor_id':3,'confirm':True})
    wait(s,lambda x:x['axes'][3]['state']=='MONITORING')
    assert {i for i,_,_ in t.sent}=={3}
    for i in (2,1):app.action('enable',{'motor_id':i,'confirm':True})
    wait(s,lambda x:all(a['state']=='MONITORING' for a in x['axes'].values()))
    assert not s.fault and len(t.receive_threads)==1
    for i in (1,2,3):assert s.get_encoder_feedback(i)['valid']
    app.action('disconnect',{})
    assert t.closed and not s._worker.is_alive() and not s._health.is_alive()
