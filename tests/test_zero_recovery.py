"""Go Zero recovery uses scripted feedback, never a physical motor."""
import math
import time
import pytest
from handycan import Session
from test_core import ScriptedTransport,wait
from test_signed_limits import configured

@pytest.mark.parametrize('position',[92,-92])
def test_zero_recovers_along_signed_path_without_changing_limits(tmp_path,position):
    motor=configured()
    with Session([motor],control_hz=100,log_path=tmp_path/'recovery.jsonl',
                 _transport_factory=ScriptedTransport,_inspector=lambda _:None) as s:
        s._transport.position=math.radians(position);s._transport.reply=True;s.enable(1)
        wait(s,lambda x:x['axes'][1]['state']=='MONITORING')
        with pytest.raises(ValueError):s.move(1,0,2,kp=2,kd=.1)
        assert s.snapshot()['fault'] is None
        s.go_zero(1,2,kp=2,kd=.1)
        with s._lock:
            path=s.axes[1]['path'];previous=path.start
            for fraction in (0,.1,.3,.5,.8,1):
                p,v=path.sample(path.began+fraction*path.duration)
                assert abs(p)<=abs(previous)+1e-12 and p*position>=-1e-12
                assert v*position<=0
                previous=p
            # Exercise runtime encoding at the outside starting point, then at
            # arrival using a small Kp so stationary test feedback stays in torque bounds.
            s._next=time.monotonic();s._tick(time.monotonic())
            from dataclasses import replace
            s.axes[1]['path']=replace(path,began=time.monotonic()-3)
            s._next=time.monotonic();s._tick(time.monotonic())
            assert s.axes[1]['state']=='HOLDING'
            assert s.axes[1]['config'].limits is motor.limits
            assert s.snapshot()['fault'] is None
        with pytest.raises(ValueError):s.move(1,position+math.copysign(1,position),2,kp=2,kd=.1)

def test_zero_recovery_keeps_speed_gains_and_feedback_checks(tmp_path):
    with Session([configured()],control_hz=100,log_path=tmp_path/'checks.jsonl',
                 _transport_factory=ScriptedTransport,_inspector=lambda _:None) as s:
        with pytest.raises(Exception):s.go_zero(1,2,kp=2,kd=.1)
        s._transport.position=math.radians(92);s._transport.reply=True;s.enable(1)
        wait(s,lambda x:x['axes'][1]['state']=='MONITORING')
        with pytest.raises(ValueError):s.go_zero(1,.01,kp=2,kd=.1)
        with pytest.raises(ValueError):s.go_zero(1,2,kp=100,kd=.1)
        assert s.snapshot()['fault'] is None
