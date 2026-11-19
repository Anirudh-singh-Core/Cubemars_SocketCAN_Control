"""RX handoff and timeout regression tests; no hardware/socket access."""
import time

import pytest

from handycan import SessionFault
from handycan.protocol import POLL
from handycan.receive import ReceiveInbox
from handycan.transport import TxConfirmation
from test_core import message, session, wait
from test_pipeline import Bench


def pending_control():
    b=Bench();b.startup();b.step(.002)
    b.p.rx_checked=lambda deadline:False
    assert b.p.control(1,POLL)
    return b,b.p.links[1]['exchange']


def test_timely_kernel_echo_survives_delayed_python_processing():
    b,e=pending_control()
    b.step(.02) # no RX drain checkpoint yet: do not invent a TX failure
    assert b.p.links[1]['exchange'] is e
    assert not any(event=='pipeline_fault' for event,_,_ in b.events)
    b.p.confirm(1,POLL,e.submitted_wall+.0003,
                captured_wall=e.submitted_wall+.0005,captured_monotonic=e.submitted+.0005)
    b.step()
    assert b.p.links[1]['exchange'] is None
    row=[data for event,_,data in b.events if event=='tx_confirmed'][-1]
    assert row['kernel_tx_delay_s']==pytest.approx(.0003)
    assert row['rx_capture_delay_s']==pytest.approx(.0002)
    assert row['rx_dispatch_delay_s']==pytest.approx(.0195)


def test_missing_echo_still_fails_after_receiver_drained_past_deadline():
    b,e=pending_control()
    checkpoint=e.submitted+.011
    b.p.rx_checked=lambda deadline:checkpoint>=deadline
    b.step(.012)
    faults=[data['reason'] for event,_,data in b.events if event=='pipeline_fault']
    assert faults and 'TX confirmation timeout after RX drain' in faults[0]


def test_late_kernel_echo_is_not_excused_by_delayed_dispatch():
    b,e=pending_control();b.step(.02)
    with pytest.raises(SessionFault,match='late TX confirmation'):
        b.p.confirm(1,POLL,e.submitted_wall+.015)


def test_unresponsive_receiver_has_bounded_explicit_failure():
    b,e=pending_control()
    b.step(.061)
    faults=[data['reason'] for event,_,data in b.events if event=='pipeline_fault']
    assert faults and 'RX processing stalled' in faults[0]


def test_waiting_for_rx_drain_does_not_busy_spin():
    b,e=pending_control();b.step(.011)
    assert b.p.wake_delay()>0


def test_confirmations_and_faults_precede_bounded_feedback_batch():
    inbox=ReceiveInbox()
    for _ in range(100):assert inbox.put(message())
    echo=TxConfirmation(1,POLL,time.time())
    fault=message(fault=3)
    assert inbox.put(echo) and inbox.put(fault)
    inbox.empty_at(123.)
    priority,feedback,checkpoint,error,backlog=inbox.take()
    assert [r.message for r in priority]==[echo,fault]
    assert len(feedback)==64 and backlog==36
    assert checkpoint==123. and error is None
    priority,feedback,checkpoint,error,backlog=inbox.take()
    assert not priority and len(feedback)==36 and not backlog


def test_inbox_overflow_is_reported_without_discarding_existing_frames():
    inbox=ReceiveInbox();inbox.CAPACITY=2
    assert inbox.put(message()) and inbox.put(message())
    assert not inbox.put(message())
    priority,feedback,checkpoint,error,backlog=inbox.take()
    assert len(feedback)==2 and 'overflow' in error


def test_receiver_captures_while_session_state_lock_is_held(session):
    # Disabled session: inject only scripted RX data, no enable/motion commands.
    # Original receiver blocks on the state lock and cannot drain this batch.
    s=session
    with s._lock:
        base=time.time()-.02
        for n in range(100):
            frame=message();frame.timestamp=base+n*.0001
            s._transport.incoming.put(frame)
        end=time.monotonic()+1
        while not s._transport.incoming.empty() and time.monotonic()<end:
            time.sleep(.001)
        assert s._transport.incoming.empty()
        # Queue.get removes the final item just before inbox.put; wait for the
        # corresponding empty checkpoint without relying on a fixed sleep.
        after=time.monotonic()
        while s._inbox._empty_through<after and time.monotonic()<end:
            time.sleep(.001)
        assert s._inbox._empty_through>=after
        assert len(s._inbox._feedback)==100
        assert not s._transport.sent
    wait(s,lambda state:state['metrics']['rx_frames']==100)
    assert not s.fault
