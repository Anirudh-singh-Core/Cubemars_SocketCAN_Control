"""Immutable degree-based samples for externally planned joint motion."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class JointTarget:
    """One absolute joint sample; gains retain MIT units, torque is Nm.

    This is a setpoint, not an IK solver or trajectory generator. The caller
    supplies smooth position/velocity samples and a complete group each update.
    """
    position_deg: float
    velocity_deg_s: float
    kp: float
    kd: float
    ff_torque: float = 0.

    def __post_init__(self):
        for value in (self.position_deg,self.velocity_deg_s,self.kp,self.kd,self.ff_torque):
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError('Joint targets must contain finite numbers')
        if not -360<=self.position_deg<=360:raise ValueError('Absolute position must be -360..360 degrees')
        if self.kp<=0:raise ValueError('Joint position streaming requires Kp > 0')

    def radians(self):
        return math.radians(self.position_deg),math.radians(self.velocity_deg_s)
