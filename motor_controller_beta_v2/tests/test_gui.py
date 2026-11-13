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
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tests'))
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


def test_gui_has_no_continuous_streaming_endpoint(app):
    s=connect(app)
    for action in ("stream_start", "stream_targets", "stream_end", "start_stream", "end_stream"):
        with pytest.raises(ValueError, match="Unknown action"):
            app.action(action,{"motor_id":1})
    assert s._transport.sent==[]
