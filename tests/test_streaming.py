"""Streaming integration checks use scripted transport only, never SocketCAN."""
import math
import time
import pytest
from handycan import Session,JointTarget,BusyError,SessionFault,can_test
from handycan.protocol import DISABLE,POLL
from test_core import ScriptedTransport,motor,wait,ready

@pytest.fixture
def session(tmp_path):
    s=Session([motor(1),motor(2)],control_hz=100,log_path=tmp_path/'stream.jsonl',
              _transport_factory=ScriptedTransport,_inspector=lambda _:None).open()
    s._transport.reply=True
    yield s
    s.close()

def monitor(s):
    for i in (1,2):s.enable(i)
    wait(s,lambda x:all(a['state']=='MONITORING' for a in x['axes'].values()))

def targets(p=0):return {i:JointTarget(p,0,2,.1) for i in (1,2)}
def start(s,policy='disable',timeout=.2):
    return s.start_stream(targets(),timeout=timeout,max_step_deg=1,timeout_action=policy)

def test_stream_requires_explicit_enable_and_configuration(session):
    with pytest.raises(BusyError):start(session)
    assert session._transport.sent==[]
    monitor(session)
    for kw in (dict(timeout=0,max_step_deg=1,timeout_action='hold'),
               dict(timeout=.2,max_step_deg=0,timeout_action='hold'),
               dict(timeout=.2,max_step_deg=1,timeout_action='resume')):
        with pytest.raises(ValueError):session.start_stream(targets(),**kw)
    assert session.snapshot()['stream'] is None

def test_latest_complete_group_replaces_pending_and_one_worker(session):
    monitor(session);token=start(session)
    wait(session,lambda x:x['stream']['applied_sequence']==0)
    with session._lock: # Deterministically prevent a tick while publishing burst.
        for n in range(1,21):session.stream_targets(token,targets(n*.001),sequence=n)
        assert session._stream['sequence']==20
        assert session._stream['replaced']>=19
        assert session._requests.empty()
    wait(session,lambda x:x['stream']['applied_sequence']==20)
    assert session._stream['applied'][1].position_deg==.02
    assert len(session._transport.receive_threads)==1
    assert len({t for _,_,t in session._transport.sent})==1
    session.end_stream(token,action='hold')
    assert all(a['state']=='HOLDING' for a in session.snapshot()['axes'].values())
    assert session.snapshot()['stream'] is None

def test_invalid_group_sequence_step_and_gain_are_atomic(session):
    monitor(session);token=start(session)
    wait(session,lambda x:x['stream']['applied_sequence']==0)
    bad=targets();bad[2]=JointTarget(0,0,100,.1)
    for group in ({1:targets()[1]},bad,targets(10),targets(330)):
        with pytest.raises(ValueError):session.stream_targets(token,group,sequence=1)
        assert session.snapshot()['stream']['sequence']==0
    session.stream_targets(token,targets(.01),sequence=1)
    with pytest.raises(ValueError):session.stream_targets(token,targets(),sequence=1)
    with pytest.raises(BusyError):session.move(1,1,1,kp=2,kd=.1)
    report=can_test(session,1,active=True,window=.05,_inspector=ready)
    assert report['result']=='FAIL'
    assert session.snapshot()['stream']['sequence']==1

def test_velocity_field_and_position_slew_checked(session):
    monitor(session);token=start(session)
    wait(session,lambda x:x['stream']['applied_sequence']==0)
    bad=targets();bad[1]=JointTarget(0,400,2,.1)
    with pytest.raises(ValueError):session.stream_targets(token,bad,sequence=1)
    with session._lock:
        session._stream['max_step_deg']=100
        session._stream['applied_at']=time.monotonic()
        with pytest.raises(ValueError,match='slew'):session.stream_targets(token,targets(4),sequence=1)

def test_timeout_disable_faults_and_cannot_be_revived(session):
    monitor(session);token=start(session,timeout=.05)
    state=wait(session,lambda x:x['fault'] is not None)
    assert 'Streaming command deadline' in state['fault']
    assert state['stream'] is None
    assert all(a['state']=='DISABLED' for a in state['axes'].values())
    with pytest.raises(SessionFault):session.stream_targets(token,targets(),sequence=1)
    assert {i for i,p,_ in session._transport.sent if p==DISABLE}=={1,2}

def test_timeout_hold_freezes_applied_position_zero_velocity(session):
    monitor(session);token=start(session,'hold',.05)
    state=wait(session,lambda x:x['stream'] is None)
    assert state['fault'] is None
    assert all(a['state']=='HOLDING' for a in state['axes'].values())
    assert all(a['path'].sample(time.monotonic())[1]==0 for a in session.axes.values())
    with pytest.raises(BusyError):session.stream_targets(token,targets(),sequence=1)
    assert not any(p==DISABLE for _,p,_ in session._transport.sent)

def test_late_publisher_cannot_reset_deadline_before_worker_checks(session):
    monitor(session);token=start(session)
    with session._lock:
        session._stream['received']=time.monotonic()-1
        with pytest.raises(SessionFault):session.stream_targets(token,targets(),sequence=1)
        assert session._stream['sequence']==0
    wait(session,lambda x:x['fault'] is not None)

def test_disable_member_cancels_whole_group_stop_invalidates_token(session):
    monitor(session);token=start(session)
    session.disable(1)
    assert session.snapshot()['stream'] is None
    assert all(a['state']=='DISABLED' for a in session.snapshot()['axes'].values())
    with pytest.raises(BusyError):session.stream_targets(token,targets(),sequence=1)
    monitor(session);new=start(session)
    session.stop();wait(session,lambda x:x['stream'] is None)
    with pytest.raises((BusyError,SessionFault)):session.stream_targets(new,targets(),sequence=1)

def test_feedback_failure_overrides_hold_policy(session):
    monitor(session);start(session,'hold',timeout=1)
    session._transport.reply=False
    state=wait(session,lambda x:x['fault'] is not None)
    assert 'stale' in state['fault'].lower()
    assert all(a['state']=='DISABLED' for a in state['axes'].values())

def test_group_preflight_sends_nothing_when_later_member_invalid(session):
    monitor(session);start(session)
    wait(session,lambda x:x['stream']['applied_sequence']==0)
    with session._lock:
        # Inject a runtime invalid command after admission to exercise tick's
        # all-member preflight; real feedback changes can invalidate torque too.
        session._stream['pending']={1:JointTarget(.01,0,2,.1),2:JointTarget(0,0,100,.1)}
        before=len(session._transport.sent)
        session._next=time.monotonic()
        with pytest.raises(ValueError):session._tick(time.monotonic())
        assert len(session._transport.sent)==before
        session._stream['pending']=targets()
