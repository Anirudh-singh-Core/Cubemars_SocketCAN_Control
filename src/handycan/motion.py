"""Pure rest-to-rest quintic profile, radians internally."""
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Trajectory:
    start: float
    target: float
    duration: float
    began: float
    kp: float
    kd: float
    torque: float
    zero_recovery: bool = False
    def sample(self,now):
        u=min(1.,max(0.,(now-self.began)/self.duration))
        delta=self.target-self.start
        return (self.start+delta*(10*u**3-15*u**4+6*u**5),
                delta*(30*u**2-60*u**3+30*u**4)/self.duration)
