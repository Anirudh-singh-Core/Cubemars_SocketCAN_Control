"""Offline tests only. A scripted transport is not a motor simulator or hardware test."""
import json
import math
import queue
import threading
import time
import can
import pytest
from handycan import Motor, Session, can_test, OwnershipError, SessionFault, FeedbackError
from handycan.protocol import POLL,ENABLE,DISABLE,SET_ZERO,encode_mit_command,decode
from handycan.transport import ChannelLease,Transport
from handycan.interface import inspect_interface


def motor(i=1):
    return Motor.from_degrees(i,joint_min=-360,joint_max=360,speed_max=280,torque_max=5,kp_max=50,kd_max=5)

def message(i=1,position=0,fault=0):
    p=int((position+12.5)*65535/25);v=2047;t=2047
    return can.Message(arbitration_id=i,is_extended_id=False,is_rx=True,timestamp=time.time(),
        data=[i,p>>8,p&255,v>>4,((v&15)<<4)|(t>>8),t&255,25,fault])

class ScriptedTransport:
    def __init__(self,channel):
        self.channel=channel;self.sent=[];self.incoming=queue.Queue();self.closed=False
        self.reply=False;self.reject=None;self.position=0;self.receive_threads=set()
    def open(self):pass
    def send(self,i,payload):
        if payload==self.reject:raise can.CanOperationError('Injected send failure')
        self.sent.append((i,payload,threading.get_ident()))
        if self.reply and payload not in (DISABLE,SET_ZERO):self.incoming.put(message(i,self.position))
    def receive(self,timeout):
        self.receive_threads.add(threading.get_ident())
        try:return self.incoming.get(timeout=timeout)
        except queue.Empty:return None
    def close(self):self.closed=True

@pytest.fixture
def session(tmp_path):
    s=Session([motor()],control_hz=100,log_path=tmp_path/'history.jsonl',
              _transport_factory=ScriptedTransport,_inspector=lambda _:None).open()
    yield s
    s.close()

def wait(s,predicate,timeout=2):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate(s.snapshot()):return s.snapshot()
        time.sleep(.002)
    raise AssertionError(s.snapshot())

def ready(channel):
    return dict(channel=channel,ready=True,reasons=[],state='ERROR-ACTIVE',counters={})

def test_protocol_matches_existing_and_rejects_echo():
    assert encode_mit_command(0,0,0,0,0)==POLL
    m=message();assert decode(m,1,1,'can0',1).raw_count==32767
    for field in ('is_extended_id','is_remote_frame','is_error_frame','is_fd'):
        m=message();setattr(m,field,True);assert decode(m,1,1,'can0',1) is None
    m=message();m.is_rx=False;assert decode(m,1,1,'can0',1) is None
    assert decode(message(2),1,1,'can0',1) is None

def test_no_implicit_motor_actions(session):
    assert session._transport.sent==[]
    transport=session._transport;session.close();assert transport.sent==[] and transport.closed

def test_shared_session_enable_disable_zero_and_filters(session):
    session._transport.reply=True
    session.enable(1);wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    raw=session.get_encoder_feedback(1)
    assert raw['source']=='raw'
    with pytest.raises(ValueError):session.set_filter(1,'kalman')
    session.set_filter(1,'kalman',rest_variance_counts_squared=1)
    wait(session,lambda _:session.axes[1]['filtered'] is not None)
    assert session.get_encoder_feedback(1,source='kalman')['valid']
    with pytest.raises(ValueError):session.set_zero(1,confirm=True)
    session.disable(1);session.set_zero(1,confirm=True)
    assert session.axes[1]['filter_name']=='raw'
    session.enable(1);wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    assert len(session._transport.receive_threads)==1
    assert len({t for _,_,t in session._transport.sent})==1

def test_stale_feedback_stops_without_retry(session):
    session._transport.reply=True;session.enable(1)
    wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    session._transport.reply=False
    result=wait(session,lambda s:s['fault'] is not None)
    assert 'stale' in result['fault']
    assert session._transport.sent[-1][1]==DISABLE
    with pytest.raises(SessionFault):session.enable(1)
    with pytest.raises(SessionFault):session.get_encoder_feedback(1)

def test_initial_no_reply_times_out(session):
    session.enable(1)
    result=wait(session,lambda s:s['fault'] is not None)
    assert 'initial' in result['fault']
    assert session._transport.sent[-1][1]==DISABLE

def test_enable_uncertain_delivery_attempts_disable(session):
    session._transport.reject=ENABLE
    with pytest.raises(can.CanError):session.enable(1)
    assert session.snapshot()['fault']
    assert session._transport.sent[-1][1]==DISABLE

def test_driver_fault_and_bus_off(session):
    session._transport.reply=True;session.enable(1)
    wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    session._transport.incoming.put(message(fault=3))
    assert 'fault byte' in wait(session,lambda s:bool(s['fault']))['fault']

def test_bus_off_error_frame_stops(session):
    session._transport.reply=True;session.enable(1)
    session._transport.incoming.put(can.Message(arbitration_id=0x40,is_error_frame=True,data=bytes(8)))
    assert 'BUS-OFF' in wait(session,lambda s:bool(s['fault']))['fault']

def test_same_channel_lock(tmp_path):
    a=ChannelLease('can0',tmp_path);b=ChannelLease('can0',tmp_path)
    a.acquire()
    try:
        with pytest.raises(OwnershipError):b.acquire()
    finally:a.close()
    b.acquire();b.close()

def test_transport_uses_python_can_and_releases_lease(tmp_path,monkeypatch):
    monkeypatch.setattr('handycan.transport.Path.home',lambda:tmp_path)
    opened=[]
    class Bus:
        def send(self,msg,timeout):opened.append((msg,timeout))
        def recv(self,timeout):return None
        def shutdown(self):opened.append('closed')
    def factory(**kw):
        assert kw['interface']=='socketcan' and kw['ignore_config'] is True
        assert not kw['receive_own_messages'];return Bus()
    monkeypatch.setattr('handycan.transport.can.Bus',factory)
    t=Transport('can0');t.open();t.send(1,POLL);t.close()
    assert isinstance(opened[0][0],can.Message) and opened[0][1]==0 and opened[-1]=='closed'
    a=ChannelLease('can0');a.acquire();a.close()

def test_move_requires_fresh_feedback_and_limits_and_holds(session):
    with pytest.raises(Exception):session.move(1,-30,1,kp=2,kd=.1)
    session._transport.reply=True;session.enable(1)
    wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    with pytest.raises(ValueError):session.move(1,400,1,kp=2,kd=.1)
    # A tiny target keeps scripted stationary feedback within the torque estimate.
    session.move(1,-.1,.1,kp=2,kd=.1)
    wait(session,lambda s:s['axes'][1]['state']=='HOLDING')
    assert all(p!=DISABLE for _,p,_ in session._transport.sent)
    session.stop();wait(session,lambda s:s['axes'][1]['state']=='DISABLED')

def test_passive_diagnostic_never_transmits(session):
    report=can_test(session,1,window=.05,_inspector=ready)
    assert report['result']=='FAIL' and not report['communication_passed']
    assert session._transport.sent==[]

def test_active_diagnostic_uses_same_socket_and_records_phases(session):
    transport=session._transport;transport.reply=True
    report=can_test(session,1,active=True,window=.08,_inspector=ready)
    assert report['result']=='PASS',report
    assert [p['name'] for p in report['phases']]==['baseline','enable','polling','disable','after_disable']
    assert transport is session._transport
    assert session.snapshot()['axes'][1]['state']=='DISABLED'
    assert len(transport.receive_threads)==1

def test_diagnostic_locates_error_phase(session):
    session._transport.reply=True
    calls=0
    def inspect(channel):
        nonlocal calls
        calls+=1
        result=ready(channel)
        result['counters']={'bus_error':4 if calls>=4 else 0}
        return result
    report=can_test(session,1,active=True,window=.08,_inspector=inspect)
    assert report['result']=='FAIL' and report['communication_passed']
    assert report['phases'][1]['counter_delta']['bus_error']==4
    assert any(reason.startswith('enable: new') for reason in report['failure_reasons'])

def test_interface_fail_closed(monkeypatch):
    for state in ('BUS-OFF','ERROR-PASSIVE','ERROR-WARNING','STOPPED','UNKNOWN'):
        data=[dict(flags=['UP'],linkinfo=dict(info_kind='can',info_data=dict(state=state,bittiming={'bitrate':1000000},ctrlmode=[])))]
        monkeypatch.setattr('handycan.interface.subprocess.check_output',lambda *a,**kw:json.dumps(data))
        assert not inspect_interface()['ready']

def test_filtered_feedback_never_changes_raw_limits(session):
    session._transport.reply=True;session.enable(1)
    wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    session.set_filter(1,'lowpass',rest_variance_counts_squared=1,cutoff=10)
    wait(session,lambda _:session.axes[1]['filtered'] is not None)
    assert session.get_encoder_feedback(1,source='lowpass')['source']=='lowpass'
    raw=session.axes[1]['feedback'].measurement
    assert raw.position_rad==pytest.approx(32767*25/65535-12.5)
    session.disable(1);time.sleep(.12)
    with pytest.raises(FeedbackError):session.get_encoder_feedback(1,source='lowpass')

def test_competing_sender_blocks_active_test_before_enable(session):
    msg=message();msg.is_rx=False
    session._transport.incoming.put(msg)
    # Continuously inject a local echo in baseline to avoid timing assumptions.
    stop=threading.Event()
    def inject():
        while not stop.wait(.01):session._transport.incoming.put(msg)
    thread=threading.Thread(target=inject);thread.start()
    try:report=can_test(session,1,active=True,window=.06,_inspector=ready)
    finally:stop.set();thread.join()
    assert report['result']=='FAIL'
    assert session._transport.sent==[]
    assert any('competing local' in r for r in report['failure_reasons'])

def test_diagnostic_exclusion_and_stop(session):
    session._transport.reply=True
    result=[]
    thread=threading.Thread(target=lambda:result.append(can_test(session,1,active=True,window=.2,_inspector=ready)))
    thread.start();time.sleep(.03)
    from handycan import BusyError
    with pytest.raises(BusyError):session.enable(1)
    session.stop();thread.join(2)
    assert not thread.is_alive() and result[0]['result']=='FAIL'
    assert all(p!=ENABLE for _,p,_ in session._transport.sent)

def test_two_motors_share_worker_and_transport(tmp_path):
    s=Session([motor(1),motor(2)],control_hz=100,log_path=tmp_path/'two.jsonl',
              _transport_factory=ScriptedTransport,_inspector=lambda _:None).open()
    try:
        s._transport.reply=True;s.enable(1);s.enable(2)
        wait(s,lambda x:all(a['state']=='MONITORING' for a in x['axes'].values()))
        assert s.get_encoder_feedback(1)['motor_id']==1
        assert s.get_encoder_feedback(2)['motor_id']==2
        assert len(s._transport.receive_threads)==1
        assert len({t for _,_,t in s._transport.sent})==1
    finally:s.close()

def test_old_queued_feedback_is_not_accepted_after_enable(session):
    old=message();old.timestamp=time.time()-2
    session.enable(1);session._transport.incoming.put(old)
    time.sleep(.03)
    assert session.snapshot()['axes'][1]['state']=='ENABLING'
    with pytest.raises(FeedbackError):session.get_encoder_feedback(1)

def test_closed_session_cannot_be_reused(session):
    session.close()
    with pytest.raises(SessionFault):session.open()
    with pytest.raises(SessionFault):session.enable(1)

def test_failed_disable_is_not_reported_as_disabled(session):
    session._transport.reply=True;session.enable(1)
    wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    session._transport.reject=DISABLE
    with pytest.raises(can.CanError):session.disable(1)
    state=session.snapshot()
    assert state['axes'][1]['state']=='FAULT' and not state['axes'][1]['disable_submitted']

def test_signed_limits_do_not_alias_equivalent_angles():
    m=Motor.from_degrees(1,joint_min=-60,joint_max=0,speed_max=360,torque_max=5,kp_max=20,kd_max=5)
    m.limits.check_quintic(0,math.radians(-30),1)
    with pytest.raises(ValueError):m.limits.check_quintic(0,math.radians(330),3)

def test_transport_open_failure_releases_lease(tmp_path,monkeypatch):
    monkeypatch.setattr('handycan.transport.Path.home',lambda:tmp_path)
    def fail(**kw):raise can.CanInitializationError('Injected initialization failure')
    monkeypatch.setattr('handycan.transport.can.Bus',fail)
    with pytest.raises(can.CanError):Transport('can0').open()
    a=ChannelLease('can0');a.acquire();a.close()

def test_history_written_and_contains_input_output(session):
    session._transport.reply=True;session.enable(1)
    wait(session,lambda s:s['axes'][1]['state']=='MONITORING')
    session.close()
    events=[json.loads(line)['event'] for line in session.log_path.read_text().splitlines()]
    assert {'request','tx','feedback','session_open'}<=set(events)

@pytest.mark.parametrize('bad_id',[True,1.0,'1',0,999])
def test_invalid_motor_id_never_sends(session,bad_id):
    with pytest.raises(ValueError):session.enable(bad_id)
    assert session._transport.sent==[]

def test_health_monitor_fault_invalidates_session(session):
    session._transport.reply=True;session.enable(1)
    session.wait_for_feedback(1)
    def failed(_):raise RuntimeError('BUS-OFF')
    session._inspector=failed
    state=wait(session,lambda s:bool(s['fault']))
    assert 'BUS-OFF' in state['fault']
    assert session._transport.sent[-1][1]==DISABLE

def test_adaptive_filter_uses_supplied_variance(session):
    session._transport.reply=True;session.enable(1);session.wait_for_feedback(1)
    session.set_filter(1,'adaptive_ema',rest_variance_counts_squared=4,min_hz=2,max_hz=40)
    wait(session,lambda _:session.axes[1]['filtered'] is not None)
    assert session.axes[1]['filter'].sigma==2
    assert session.get_encoder_feedback(1,source='adaptive_ema')['valid']
