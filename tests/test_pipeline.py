"""Deterministic sequencing/backpressure checks; never open a CAN interface."""
from concurrent.futures import Future
import errno
import socket
import struct
import time

import can
import pytest

from handycan import Session, SessionFault, TimingPolicy, JointTarget, can_test
from handycan.pipeline import Pipeline
from handycan.protocol import POLL, ENABLE, DISABLE
from handycan.transport import Transport, decode_datagram, TxConfirmation, TIMESPEC, SO_TIMESTAMPNS, SO_RXQ_OVFL
from test_core import ScriptedTransport, motor, ready, wait


class Bench:
    def __init__(self,ids=(1,2),hz=1000,timing=None):
        self.now=10.;self.sent=[];self.events=[];self.busy=False
        self.p=Pipeline(ids,1/hz,self.send,self.event,timing=timing,clock=lambda:self.now,wall=lambda:self.now)
    def send(self,i,payload,kind):
        if self.busy:raise BlockingIOError(errno.ENOBUFS,'full')
        self.sent.append((i,payload,kind,self.now))
    def event(self,event,i,**data):
        self.events.append((event,i,data))
        if event=='pipeline_fault':self.p.disable(i)
    def step(self,dt=0):self.now+=dt;self.p.advance()
    def confirm(self,i,feedback=False):
        e=self.p.links[i]['exchange'];assert e and e.submitted is not None
        self.p.confirm(i,e.payload,self.now)
        if feedback:self.p.feedback(i,self.now)
    def startup(self,i=1):
        f=Future();self.p.start(i,f,.1)
        self.step();self.step();self.confirm(i) # neutral may have no reply
        self.step(.0101);self.step();self.confirm(i) # enable may have no reply
        self.step(.0101);self.step();self.confirm(i,True) # explicit probe
        self.step()
        assert f.result()['feedback_verified'];return f


@pytest.mark.parametrize('hz',[500,1000])
def test_startup_never_bursts_and_first_poll_waits_full_period(hz):
    b=Bench(hz=hz);b.startup()
    assert [p for _,p,_,_ in b.sent]==[POLL,ENABLE,POLL]
    assert all(y[3]-x[3]>=.01 for x,y in zip(b.sent,b.sent[1:]))
    assert not b.p.control(1,POLL)
    b.step(1/hz-.00001);assert not b.p.control(1,POLL)
    b.step(.00002);assert b.p.control(1,POLL)
    b.step();assert len(b.sent)==4


def test_neutral_reply_cannot_complete_enable_or_probe():
    b=Bench();f=Future();b.p.start(1,f,.1);b.step();b.step()
    b.confirm(1,True);b.step(.0101);b.step();b.confirm(1,True)
    b.step(.0101);b.step();b.confirm(1)
    assert not f.done()
    e=b.p.links[1]['exchange']
    b.p.feedback(1,e.submitted_wall-1);b.step();assert not f.done()
    b.p.feedback(1,b.now);b.step();assert f.result()['feedback_verified']


def test_startups_are_serialized_even_for_concurrent_requests():
    b=Bench();f1=Future();f2=Future()
    b.p.start(1,f1,.1);b.p.start(2,f2,.1)
    b.step();b.step()
    assert [i for i,_,_,_ in b.sent]==[1]
    for dt in (.0101,.0101):
        b.confirm(1,True);b.step(dt);b.step()
        assert all(i==1 for i,_,_,_ in b.sent)
    b.confirm(1,True);b.step();b.step()
    assert f1.done() and not f2.done() and b.sent[-1][0]==2


def test_feedback_without_tx_confirmation_does_not_advance():
    b=Bench();f=Future();b.p.start(1,f,.1);b.step();b.step()
    b.p.feedback(1,b.now);b.step(.0101)
    with pytest.raises(SessionFault,match='TX confirmation'):f.result()
    assert len(b.sent)==1


def test_control_waits_only_for_tx_confirmation_not_motor_feedback():
    b=Bench();b.startup();b.step(.002)
    assert b.p.control(1,POLL);b.step()
    for _ in range(3):
        b.step(.001);assert not b.p.control(1,b'12345678')
    assert len(b.sent)==4
    b.confirm(1);b.step() # no motor reply
    assert b.p.control(1,b'87654321');b.step()
    assert b.sent[-1][1]==b'87654321'


def test_disable_cancels_unsent_target_and_waits_for_sent_exchange():
    b=Bench();b.startup();b.step(.002)
    b.busy=True;assert not b.p.control(1,b'12345678');b.busy=False
    f=Future();b.p.disable(1,f)
    b.step();b.step()
    assert b.sent[-1][1]==DISABLE
    assert all(p!=b'12345678' for _,p,_,_ in b.sent)
    assert not f.done();b.confirm(1);b.step(.0101)
    assert f.result()['transmission_confirmed'] and not f.result()['execution_verified']
    assert not b.p.control(1,POLL)

    b=Bench();b.startup();b.step(.002);b.p.control(1,POLL);b.step()
    f=Future();b.p.disable(1,f);b.step()
    assert b.sent[-1][1]!=DISABLE
    b.confirm(1,True);b.step();b.step();assert b.sent[-1][1]==DISABLE


def test_tx_busy_has_deadline_and_does_not_claim_submission():
    b=Bench();b.busy=True;f=Future();b.p.start(1,f,.1);b.step();b.step()
    assert not b.sent and b.p.links[1]['exchange'].submitted is None
    b.step(.011)
    with pytest.raises(SessionFault,match='TX queue deadline'):f.result()


def test_temporary_tx_busy_retries_only_unsent_packet():
    b=Bench();b.busy=True;f=Future();b.p.start(1,f,.1);b.step();b.step()
    b.step(.001);b.busy=False;b.step()
    assert len(b.sent)==1
    b.step(.001);assert len(b.sent)==1 # awaiting confirmation, no retransmit


def test_late_tx_confirmation_is_not_accepted_as_success():
    b=Bench();f=Future();b.p.start(1,f,.1);b.step();b.step()
    with pytest.raises(SessionFault,match='late TX'):
        b.p.confirm(1,POLL,b.now+.02)


def test_disable_confirmation_failure_is_explicit():
    b=Bench();b.startup();f=Future();b.p.disable(1,f);b.step();b.step();b.step(.011)
    with pytest.raises(SessionFault,match='confirmation'):f.result()
    assert b.p.links[1]['stage']=='failed'


def test_stop_cancels_unsent_zero_and_shutdown_finishes_waiters():
    b=Bench();f=Future();b.p.zero(1,f);b.p.cancel_zero(1);b.step()
    assert b.sent==[]
    with pytest.raises(SessionFault,match='interrupted'):f.result()
    f=Future();b.p.zero(1,f);b.step();b.p.finish_shutdown()
    with pytest.raises(SessionFault,match='closed'):f.result()


def test_new_motion_cancels_old_unsent_target_but_cannot_recall_submitted_target():
    b=Bench();b.startup();b.step(.002)
    b.busy=True;assert not b.p.control(1,b'12345678');b.p.cancel_unsent_control(1)
    assert b.p.links[1]['exchange'] is None
    b.busy=False
    b.p.control(1,b'87654321');b.step();b.p.cancel_unsent_control(1)
    assert b.p.links[1]['exchange'] is not None
    assert b.sent[-1][1]==b'87654321'


def test_out_of_order_or_foreign_confirmation_cannot_clear_exchange():
    b=Bench();f=Future();b.p.start(1,f,.1);b.step();b.step()
    b.p.confirm(2,POLL,b.now);b.p.confirm(1,ENABLE,b.now);b.p.confirm(1,POLL,b.now-1)
    assert b.p.links[1]['exchange'].confirmed is None
    assert not f.done()


def datagram(flags=0,ancillary=None,ident=1):
    frame=struct.pack('=IB3x8s',ident,8,b'12345678')
    if ancillary is None:ancillary=[(socket.SOL_SOCKET,SO_TIMESTAMPNS,TIMESPEC.pack(10,123))]
    return decode_datagram(frame,ancillary,flags,'can0')


def test_transport_distinguishes_own_confirmation_other_sender_and_external_feedback():
    assert isinstance(datagram(socket.MSG_CONFIRM|socket.MSG_DONTROUTE),TxConfirmation)
    assert datagram(socket.MSG_DONTROUTE).is_rx is False
    assert datagram().is_rx is True
    assert datagram(ident=0x20000088).is_error_frame
    assert datagram(ident=0x20000088).arbitration_id==0x88


@pytest.mark.parametrize('flags',[socket.MSG_TRUNC,socket.MSG_CTRUNC])
def test_transport_rejects_truncation(flags):
    with pytest.raises(can.CanOperationError):datagram(flags)


def test_transport_detects_receive_loss_and_missing_timestamps():
    with pytest.raises(can.CanOperationError,match='overflow'):
        datagram(ancillary=[(socket.SOL_SOCKET,SO_RXQ_OVFL,struct.pack('=I',3))])
    with pytest.raises(can.CanOperationError,match='timestamp'):datagram(ancillary=[])


def test_transport_requires_driver_echo(monkeypatch):
    monkeypatch.setattr('handycan.transport.Path.read_text',lambda _: '0x81')
    with pytest.raises(can.CanInitializationError,match='IFF_ECHO'):Transport('can0')._require_driver_echo()
    monkeypatch.setattr('handycan.transport.Path.read_text',lambda _: '0x40081')
    Transport('can0')._require_driver_echo()


def test_python_can_busy_error_remains_retryable():
    class Bus:
        def send(self,*args,**kwargs):raise can.CanOperationError('busy',errno.ENOBUFS)
    t=Transport('can0');t.bus=Bus()
    with pytest.raises(BlockingIOError) as exc:t.send(1,POLL)
    assert exc.value.errno==errno.ENOBUFS


@pytest.mark.parametrize('ids,hz',[
    ((1,2),500),((2,1),500),((1,2),1000),((2,1),1000),
    ((1,2,3),500),((3,2,1),500),((1,2,3),800),((3,2,1),800)])
def test_repeated_multimotor_diagnostic_same_session(tmp_path,ids,hz):
    with Session([motor(i) for i in ids],control_hz=hz,log_path=tmp_path/'test.jsonl',
                 _transport_factory=ScriptedTransport,_inspector=lambda _:None) as s:
        s._transport.reply=True
        for _ in range(2):
            report=can_test(s,active=True,window=.06,_inspector=ready)
            assert report['result']=='PASS',report
            assert all(a['state']=='DISABLED' for a in s.snapshot()['axes'].values())
        assert len(s._transport.receive_threads)==1


def test_three_motors_at_1000_hz_still_respects_existing_bandwidth_limit():
    with pytest.raises(ValueError,match='80%'):
        Session([motor(1),motor(2),motor(3)],control_hz=1000)


def test_stop_interrupts_startup_without_later_enable(tmp_path):
    import threading
    with Session([motor()],control_hz=500,log_path=tmp_path/'stop.jsonl',
                 _transport_factory=ScriptedTransport,_inspector=lambda _:None) as s:
        errors=[]
        def enable():
            try:s.enable(1)
            except SessionFault as exc:errors.append(exc)
        thread=threading.Thread(target=enable);thread.start()
        wait(s,lambda x:x['axes'][1]['state']=='ENABLING')
        s.stop();thread.join(2)
        wait(s,lambda x:x['axes'][1]['state']=='DISABLED')
        n=len(s._transport.sent);time.sleep(.02)
        assert len(s._transport.sent)==n and errors and not thread.is_alive()


def test_diagnostic_error_at_second_enable_stops_before_third_motor(tmp_path):
    class BrokenSecond(ScriptedTransport):
        def send(self,i,payload):
            super().send(i,payload)
            if i==2 and payload==ENABLE:
                self.incoming.put(can.Message(arbitration_id=0x88,is_extended_id=False,
                    is_error_frame=True,timestamp=time.time(),data=[0,0,16,0,0,0,0,0]))
    with Session([motor(1),motor(2),motor(3)],control_hz=500,log_path=tmp_path/'failure.jsonl',
                 _transport_factory=BrokenSecond,_inspector=lambda _:None) as s:
        s._transport.reply=True
        report=can_test(s,active=True,window=.06,_inspector=ready)
        assert report['result']=='FAIL'
        assert any(p['name']=='motor_2/enable' and p['error_frame_delta']==1 for p in report['phases'])
        assert all(i!=3 for i,_,_ in s._transport.sent)
        assert '0x88' in s.fault


def test_late_tick_and_repeated_queue_busy_never_replay_old_control():
    b=Bench();b.startup();b.step(.003)
    b.busy=True
    for _ in range(20):
        assert not b.p.control(1,b'old!!!!!')
        b.step(.004)
    assert b.p.links[1]['exchange'] is None
    assert not any(event=='pipeline_fault' for event,_,_ in b.events)
    b.busy=False
    assert b.p.control(1,b'latest!!')
    assert b.sent[-1][1]==b'latest!!'
    assert all(payload!=b'old!!!!!' for _,payload,_,_ in b.sent)


def test_sequential_deadlines_use_explicit_policy():
    b=Bench(timing=TimingPolicy(special_tx_queue_timeout_s=.03))
    b.busy=True;f=Future();b.p.start(1,f,.1);b.step();b.step();b.step(.02)
    assert not f.done()
    b.step(.011)
    with pytest.raises(SessionFault,match='TX queue'):f.result()
    b=Bench(timing=TimingPolicy(tx_confirmation_timeout_s=.03))
    f=Future();b.p.start(1,f,.1);b.step();b.step();b.now+=.02;b.confirm(1)
    assert b.p.links[1]['exchange'].confirmed is not None


@pytest.mark.parametrize('kwargs',[
    {'startup_feedback_timeout_s':float('nan')},{'tx_confirmation_timeout_s':0},
    {'scheduler_stall_s':True},{'api_timeout_s':.1},{'shutdown_timeout_s':.001}])
def test_invalid_timing_policy_rejected_without_io(kwargs):
    with pytest.raises(ValueError):TimingPolicy(**kwargs)


def test_cli_configuration_loads_one_timing_snapshot(tmp_path):
    import json
    from handycan.cli import load_config
    p=tmp_path/'config.json'
    row=dict(motor_id=1,joint_min=-90,joint_max=90,speed_max=280,torque_max=5,kp_max=50,kd_max=5,
             feedback_timeout=.2)
    p.write_text(json.dumps(dict(motors=[row],control_hz=500,
        timing={'startup_feedback_timeout_s':.8,'special_response_window_s':.03})))
    motors,channel,hz,timing=load_config(p)
    assert channel=='can0' and hz==500 and motors[0].limits.feedback_max_age_s==.2
    assert timing.startup_feedback_timeout_s==.8 and timing.special_response_window_s==.03


@pytest.mark.parametrize('streaming',[False,True])
def test_missing_reply_does_not_gate_transmission_before_freshness_limit(tmp_path,streaming):
    with Session([motor(1),motor(2)],control_hz=500,log_path=tmp_path/'async.jsonl',
                 _transport_factory=ScriptedTransport,_inspector=lambda _:None) as s:
        s._transport.reply=True;s.enable(1);s.enable(2)
        if streaming:
            s.start_stream({i:JointTarget(0,0,1,.1) for i in (1,2)},timeout=.5,
                           max_step_deg=1,timeout_action='disable')
        s._transport.reply=False
        before={i:a['control_submitted'] for i,a in s.snapshot()['axes'].items()}
        state=wait(s,lambda x:all(x['axes'][i]['control_submitted']>=before[i]+4 for i in (1,2)))
        assert state['fault'] is None
        assert s._receiver.ident!=s._worker.ident
        assert s._transport.receive_threads=={s._receiver.ident}
        assert {thread for _,_,thread in s._transport.sent}=={s._worker.ident}
        assert 'stale' in wait(s,lambda x:bool(x['fault']))['fault']
    assert not s._receiver.is_alive() and not s._worker.is_alive()


def test_partial_stream_submission_recovers_with_latest_group(tmp_path):
    class BusySecond(ScriptedTransport):
        blocked=False
        def send(self,i,payload):
            if self.blocked and i==2 and payload not in (ENABLE,DISABLE):
                raise BlockingIOError(errno.ENOBUFS,'scripted queue busy')
            super().send(i,payload)
    with Session([motor(1),motor(2)],control_hz=500,log_path=tmp_path/'partial.jsonl',
                 _transport_factory=BusySecond,_inspector=lambda _:None) as s:
        s._transport.reply=True;s.enable(1);s.enable(2)
        def targets(p):return {i:JointTarget(p,0,1,.1) for i in (1,2)}
        token=s.start_stream(targets(0),timeout=.5,max_step_deg=1,timeout_action='disable')
        wait(s,lambda x:x['stream']['applied_sequence']==0)
        s._transport.blocked=True
        s.stream_targets(token,targets(.01),sequence=1)
        wait(s,lambda x:x['metrics']['control_samples_dropped']>0)
        assert s.snapshot()['stream']['applied_sequence']==0
        s.stream_targets(token,targets(.02),sequence=2)
        s._transport.blocked=False
        state=wait(s,lambda x:x['stream']['applied_sequence']==2)
        assert state['fault'] is None and s._stream['applied'][2].position_deg==.02
