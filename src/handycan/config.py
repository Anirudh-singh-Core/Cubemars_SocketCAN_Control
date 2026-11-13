"""Explicit motor identity and caller-provided degree limits; no implicit activation."""
from dataclasses import dataclass, replace
import math
from .limits import MotorLimits
from .protocol import encode_mit_command
from .interface import validate_channel

@dataclass(frozen=True)
class Motor:
    limits: MotorLimits
    reply_id: int | None=None
    def __post_init__(self):
        if not isinstance(self.limits,MotorLimits):raise TypeError('MotorLimits required')
        if self.reply_id is None:object.__setattr__(self,'reply_id',self.limits.motor_id)
        if isinstance(self.reply_id,bool) or not isinstance(self.reply_id,int) or not 0<=self.reply_id<=2047:
            raise ValueError('Reply ID must be 0..2047')
    @classmethod
    def from_degrees(cls,motor_id,*,joint_min,joint_max,speed_max,torque_max,kp_max,kd_max,
                     channel='can0',reply_id=None,feedback_timeout=.1):
        values=(joint_min,joint_max,speed_max,torque_max,kp_max,kd_max,feedback_timeout)
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in values):
            raise ValueError('Motor configuration values must be finite numbers')
        if not -360<=joint_min<joint_max<=360 or speed_max<=0 or torque_max<=0:
            raise ValueError('Require -360 <= joint_min < joint_max <= 360 and positive speed/torque magnitudes')
        return cls(MotorLimits(motor_id,math.radians(joint_min),math.radians(joint_max),
            -math.radians(speed_max),math.radians(speed_max),-torque_max,torque_max,
            kp_max=kp_max,kd_max=kd_max,channel=channel,feedback_max_age_s=feedback_timeout,
            position_tolerance_rad=25/65535,enforce_measured_position=False,position_period_rad=None),reply_id)

def validate(motors,channel,hz):
    validate_channel(channel)
    if not motors or len({m.limits.motor_id for m in motors})!=len(motors):raise ValueError('Provide unique motor IDs')
    if isinstance(hz,bool) or not math.isfinite(hz) or not 1<=hz<=1000:raise ValueError('Rate must be 1..1000 Hz')
    if len(motors)*hz*270>800000:raise ValueError('Conservative CAN traffic estimate exceeds 80% at 1 Mbps')
    for motor in motors:
        motor.limits.check_identity(motor.limits.motor_id,channel)
        encode_mit_command(0,0,0,0,0,limits=motor.limits,motor_id=motor.limits.motor_id,channel=channel)
