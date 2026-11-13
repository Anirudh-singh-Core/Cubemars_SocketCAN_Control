"""Signed absolute joint limits, independent of wrapped display orientation."""
import math
import time
import pytest
from handycan import Motor,Session,JointTarget
from handycan.protocol import encode_mit_command,DISABLE
from handycan.limits import MotorFeedback
from test_core import ScriptedTransport,wait


def configured(lo=-90,hi=90):
    return Motor.from_degrees(1,joint_min=lo,joint_max=hi,speed_max=280,
                             torque_max=5,kp_max=50,kd_max=5)

@pytest.mark.parametrize('lo,hi',[(-360,360),(-90,90),(-30,120),(0,360)])
def test_signed_bounds_are_linear(lo,hi):
    lim=configured(lo,hi).limits
    assert lim.position_period_rad is None
    assert math.degrees(lim.position_min_rad)==pytest.approx(lo)
    assert math.degrees(lim.position_max_rad)==pytest.approx(hi)
    lim.check_position('min',math.radians(lo));lim.check_position('max',math.radians(hi))
    for p in (lo-.001,hi+.001):
        with pytest.raises(ValueError):lim.check_position('outside',math.radians(p))

def test_wrapped_equivalents_are_not_interchangeable():
    lim=configured().limits
    lim.check_quintic(math.radians(80),math.radians(-80),2)
    lim.check_position('target',math.radians(-30))
    for p in (330,-100,360,-360):
        with pytest.raises(ValueError):lim.check_position('target',math.radians(p))
    # A literal 0..360 configuration no longer permits negative targets.
    with pytest.raises(ValueError):configured(0,360).limits.check_position('target',math.radians(-30))

def test_full_signed_travel_preserves_both_revolutions():
    lim=configured(-360,360).limits
    lim.check_quintic(math.radians(-360),math.radians(360),10)
    from handycan.motion import Trajectory
    path=Trajectory(math.radians(-360),math.radians(360),10,0,2,.1,0)
    assert math.degrees(path.sample(5)[0])==pytest.approx(0)
    assert math.degrees(path.sample(10)[0])==pytest.approx(360)

@pytest.mark.parametrize('lo,hi',[(-361,360),(-360,361),(90,-90),(0,0)])
def test_invalid_signed_configuration(lo,hi):
    with pytest.raises(ValueError):configured(lo,hi)

def test_measured_outside_warns_without_disable_and_display_retains_sign(tmp_path):
    with Session([configured(-30,30)],control_hz=100,log_path=tmp_path/'signed.jsonl',
                 _transport_factory=ScriptedTransport,_inspector=lambda _:None) as s:
        s._transport.position=math.radians(-40);s._transport.reply=True;s.enable(1)
        wait(s,lambda x:x['axes'][1]['state']=='MONITORING' and x['axes'][1]['position_warning'])
        reading=s.get_encoder_feedback(1)
        assert reading['position_deg']==pytest.approx(-40,abs=.03)
        assert reading['position_single_turn_deg']==pytest.approx(320,abs=.03)
        with pytest.raises(ValueError):s.move(1,330,5,kp=2,kd=.1)
        assert s.snapshot()['fault'] is None
        assert not any(p==DISABLE for _,p,_ in s._transport.sent)

def test_stream_rejects_wrapped_alias_without_replacing_group(tmp_path):
    with Session([configured()],control_hz=100,log_path=tmp_path/'stream.jsonl',
                 _transport_factory=ScriptedTransport,_inspector=lambda _:None) as s:
        s._transport.reply=True;s.enable(1)
        wait(s,lambda x:x['axes'][1]['state']=='MONITORING')
        token=s.start_stream({1:JointTarget(0,0,2,.1)},timeout=.2,max_step_deg=1,timeout_action='hold')
        with pytest.raises(ValueError):s.stream_targets(token,{1:JointTarget(330,0,2,.1)},sequence=1)
        assert s.snapshot()['stream']['sequence']==0
        assert s.snapshot()['fault'] is None
        s.end_stream(token,action='disable')
