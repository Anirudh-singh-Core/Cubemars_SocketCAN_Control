#!/usr/bin/env python3
"""Caller-provided software limits for AK70-10 V2 MIT commands.

Pure checks: no CAN access or motor actions. Internal units are signed radians,
rad/s and Nm. Periodic limits constrain entire paths without wrapping commands.
Requested and quantized commands are validated separately; no silent clipping.
Combined PD plus feedforward torque is an estimate, not physical torque assurance.
Optional current estimates require a caller-verified output torque constant.
After Set Zero, existing numeric limits refer to the new motor reference.
"""
from dataclasses import dataclass
import math
import time


class MotorLimitError(ValueError):
    """A requested command or feedback violates the supplied software policy."""


def _number(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise MotorLimitError(f'{name} must be a finite number')


@dataclass(frozen=True)
class MotorFeedback:
    """Fresh sample in motor-zero units; timestamp uses time.monotonic()."""
    motor_id: int
    position_rad: float
    velocity_rad_s: float
    timestamp: float
    channel: str = 'can0'
    torque_nm: float | None = None


@dataclass(frozen=True)
class MotorLimits:
    """Explicit per-call limits; protocol ranges are ceilings, not safe ratings.

    Bounds are inclusive. Position target bounds apply only with Kp>0 (the
    position field has no effect at Kp=0). Measured joint-position enforcement
    defaults on; Motor.from_degrees opts out and reports boundary warnings instead. Protocol
    validity, freshness and other measured limits are still checked. Velocity bounds are signed, e.g. -1 to +1.
    kp_max and kd_max are caller-supplied gain ceilings (defaults: 500 and 5).
    Commands must satisfy 0 <= Kp <= kp_max and 0 <= Kd <= kd_max.
    Feedback age is checked at each command; reuse a fresh sample each cycle.
    position_tolerance_rad defaults to zero. Motor.from_degrees explicitly allows one
    CAN position count for measured/encoded boundaries; requested targets stay
    strictly inside the supplied interval (or equivalent allowed arc when periodic).
    position_period_rad defaults to None for linear limits. Periodic paths may not
    cross excluded arcs or exceed one period. Torque/velocity limits stay strict.
    """
    motor_id: int
    position_min_rad: float
    position_max_rad: float
    velocity_min_rad_s: float
    velocity_max_rad_s: float
    torque_min_nm: float
    torque_max_nm: float
    kp_max: float = 500
    kd_max: float = 5
    feedback_max_age_s: float = .1
    channel: str = 'can0'
    current_max_a: float | None = None
    torque_constant_nm_per_a: float | None = None
    position_tolerance_rad: float = 0
    enforce_measured_position: bool = True
    position_period_rad: float | None = None

    protocol_velocity_max: float = 50
    protocol_torque_max: float = 25

    def __post_init__(self):
        if isinstance(self.motor_id, bool) or not isinstance(self.motor_id, int) or not 1 <= self.motor_id <= 255:
            raise MotorLimitError('motor_id must be an integer in 1..255')
        if not isinstance(self.channel, str) or not self.channel:
            raise MotorLimitError('channel must be a nonempty string')
        for name in ('protocol_velocity_max','protocol_torque_max'):
            _number(name,getattr(self,name))
            if getattr(self,name)<=0:raise MotorLimitError(name+' must be positive')
        for name, low, high, floor, ceiling in (
            ('position', self.position_min_rad, self.position_max_rad, -12.5, 12.5),
            ('velocity', self.velocity_min_rad_s, self.velocity_max_rad_s, -self.protocol_velocity_max, self.protocol_velocity_max),
            ('torque', self.torque_min_nm, self.torque_max_nm, -self.protocol_torque_max, self.protocol_torque_max),
        ):
            _number(name+'_min', low); _number(name+'_max', high)
            if not floor <= low <= high <= ceiling:
                raise MotorLimitError(f'{name}: require {floor} <= min <= max <= {ceiling}')
        for name, value, ceiling in (('kp_max',self.kp_max,500),('kd_max',self.kd_max,5)):
            _number(name,value)
            if not 0 <= value <= ceiling:
                raise MotorLimitError(f'{name} must be in 0..{ceiling}')
        if (self.current_max_a is None) != (self.torque_constant_nm_per_a is None):
            raise MotorLimitError('Supply both current_max_a and verified output torque_constant_nm_per_a, or neither')
        if self.current_max_a is not None:
            for name,value in (('current_max_a',self.current_max_a),('torque_constant_nm_per_a',self.torque_constant_nm_per_a)):
                _number(name,value)
                if value<=0:raise MotorLimitError(f'{name} must be positive')
        _number('feedback_max_age_s',self.feedback_max_age_s)
        if self.feedback_max_age_s <= 0:
            raise MotorLimitError('feedback_max_age_s must be positive')
        _number('position_tolerance_rad',self.position_tolerance_rad)
        if not 0 <= self.position_tolerance_rad <= 25/65535:
            raise MotorLimitError('Position tolerance cannot exceed one CAN position count')
        if not isinstance(self.enforce_measured_position,bool):
            raise MotorLimitError('enforce_measured_position must be boolean')
        if self.position_period_rad is not None:
            _number('position_period_rad',self.position_period_rad)
            if not 0 < self.position_period_rad <= 25 or self.position_max_rad-self.position_min_rad > self.position_period_rad:
                raise MotorLimitError('Position interval must fit inside a positive period <= 25 rad')

    def position_interval(self, position, tolerance=0):
        """Return the equivalent allowed interval containing this unwrapped angle.

        Periodic limits constrain orientation, while CAN commands/feedback retain
        their signed motor-zero coordinates. None means outside the allowed arc.
        """
        _number('position',position)
        low,high=self.position_min_rad,self.position_max_rad
        if self.position_period_rad is None:
            return (low,high) if low-tolerance<=position<=high+tolerance else None
        period=self.position_period_rad
        turn=math.floor((position-low)/period)
        for k in (turn,turn+1,turn-1):
            lo,hi=low+k*period,high+k*period
            if lo-tolerance<=position<=hi+tolerance:return lo,hi
        return None

    def check_position(self, name, position, tolerance=0):
        if self.position_interval(position,tolerance) is None:
            raise MotorLimitError(f'CAN {self.channel}/{self.motor_id}: {name}={position:g} outside '
                                 f'[{self.position_min_rad:g}, {self.position_max_rad:g}]'
                                 + (' modulo one turn' if self.position_period_rad else ''))

    def trajectory_start(self, position):
        """Clamp only an accepted boundary quantization residual, preserving turns."""
        self.check_position('trajectory start',position,self.position_tolerance_rad)
        low,high=self.position_interval(position,self.position_tolerance_rad)
        return min(high,max(low,position))

    def _bound(self, name, value, low, high):
        _number(name,value)
        if not low <= value <= high:
            raise MotorLimitError(f'CAN {self.channel}/{self.motor_id}: {name}={value:g} outside [{low:g}, {high:g}]')

    def check_identity(self, motor_id, channel):
        if isinstance(motor_id,bool) or not isinstance(motor_id,int) or (motor_id,channel) != (self.motor_id,self.channel):
            raise MotorLimitError('Limits belong to a different motor ID/channel')

    def check_feedback(self, feedback):
        """Validate feedback; measured joint bounds are conditional on policy. No action."""
        if not isinstance(feedback,MotorFeedback):
            raise MotorLimitError('Supply a MotorFeedback sample')
        self.check_identity(feedback.motor_id,feedback.channel)
        _number('feedback timestamp',feedback.timestamp)
        age=time.monotonic()-feedback.timestamp
        if not 0 <= age <= self.feedback_max_age_s:
            raise MotorLimitError('Feedback is stale or timestamp is in the future')
        self._bound('measured position protocol range',feedback.position_rad,-12.5,12.5)
        if self.enforce_measured_position:
            self.check_position('measured position',feedback.position_rad,self.position_tolerance_rad)
        self._bound('measured velocity',feedback.velocity_rad_s,self.velocity_min_rad_s,self.velocity_max_rad_s)
        if feedback.torque_nm is not None:
            self._bound('measured torque',feedback.torque_nm,self.torque_min_nm,self.torque_max_nm)
            self.check_current_estimate(feedback.torque_nm)

    def check_current_estimate(self, torque):
        """Optional torque/Kt estimate; NOT an independent measured current limit."""
        if self.current_max_a is not None:
            self._bound('estimated current (A)',abs(torque)/self.torque_constant_nm_per_a,0,self.current_max_a)

    def check_gains_torque(self, kp, kd, ff_torque):
        """Check static settings before enabling; combined torque needs feedback."""
        self._bound('Kp',kp,0,self.kp_max)
        self._bound('Kd',kd,0,self.kd_max)
        self._bound('feedforward torque',ff_torque,self.torque_min_nm,self.torque_max_nm)
        self.check_current_estimate(ff_torque)

    def check_command(self, position, velocity, kp, kd, ff_torque=0, *, motor_id, channel='can0', feedback=None, quantized=False):
        """Validate a command; return estimated torque, or FF torque at zero gains."""
        self.check_identity(motor_id,channel)
        self._bound('position field',position,-12.5,12.5)
        self.check_gains_torque(kp,kd,ff_torque)
        if kp > 0:
            tolerance=self.position_tolerance_rad if quantized else 0
            self.check_position('target position',position,tolerance)
        self._bound('target velocity',velocity,self.velocity_min_rad_s,self.velocity_max_rad_s)
        self._bound('feedforward torque',ff_torque,self.torque_min_nm,self.torque_max_nm)
        if feedback is not None:
            self.check_feedback(feedback)
        elif kp != 0 or kd != 0:
            raise MotorLimitError('Fresh MotorFeedback required to check combined MIT torque with nonzero gains')
        torque=ff_torque
        if feedback is not None:
            torque += kp*(position-feedback.position_rad)+kd*(velocity-feedback.velocity_rad_s)
        self._bound('estimated combined torque',torque,self.torque_min_nm,self.torque_max_nm)
        self.check_current_estimate(torque)
        return torque

    def check_quintic(self, start, target, duration):
        """Preflight monotonic rest-to-rest quintic position and peak velocity."""
        _number('duration',duration)
        if duration <= 0:
            raise MotorLimitError('duration must be positive')
        self._bound('trajectory start protocol range',start,-12.5,12.5)
        self._bound('trajectory target protocol range',target,-12.5,12.5)
        self.check_position('trajectory start',start,self.position_tolerance_rad)
        self.check_position('trajectory target',target)
        if self.position_period_rad is not None:
            self._bound('trajectory travel',abs(target-start),0,self.position_period_rad+self.position_tolerance_rad)
            if self.position_max_rad-self.position_min_rad < self.position_period_rad:
                low,high=self.position_interval(start,self.position_tolerance_rad)
                if not low<=target<=high:
                    raise MotorLimitError('Trajectory crosses an excluded joint-angle region')
        self._bound('endpoint velocity',0,self.velocity_min_rad_s,self.velocity_max_rad_s)
        self._bound('peak trajectory velocity',1.875*(target-start)/duration,
                    self.velocity_min_rad_s,self.velocity_max_rad_s)
