"""Pure AK70-10 V2 MIT codec; no bus access or motor actions."""
from dataclasses import dataclass
import math
from .limits import MotorLimits, MotorLimitError, MotorFeedback

ENABLE=bytes.fromhex('fffffffffffffffc')
DISABLE=bytes.fromhex('fffffffffffffffd')
SET_ZERO=bytes.fromhex('fffffffffffffffe')
POLL=bytes.fromhex('7fff7ff0000007ff')

def _uint(value, low, high, bits, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be finite and in [{low}, {high}]')
    return int((value-low)*((1 << bits)-1)/(high-low))

def encode_mit_command(position, velocity, kp, kd, ff_torque=0, *, limits=None, feedback=None, motor_id=1, channel='can0'):
    """AK70-10 V2 encoder; optional caller-supplied limits, no transmission.

    Checks both requested and quantized values. Optional position_tolerance_rad
    permits at most one CAN count at encoded boundaries; requested targets stay
    strict. Torque and velocity quantization must still fit their signed bounds.
    Nonzero gains with limits require fresh MotorFeedback for estimated torque.
    """
    if limits is not None:
        if not isinstance(limits,MotorLimits):
            raise MotorLimitError('limits must be a MotorLimits object')
        limits.check_command(position,velocity,kp,kd,ff_torque,
                             motor_id=motor_id,channel=channel,feedback=feedback)
    p = _uint(position, -12.5, 12.5, 16, 'position')
    v = _uint(velocity, -50, 50, 12, 'velocity')
    k_p = _uint(kp, 0, 500, 12, 'kp')
    k_d = _uint(kd, 0, 5, 12, 'kd')
    torque = _uint(ff_torque, -25, 25, 12, 'ff_torque')
    if limits is not None:
        limits.check_command(p*25/65535-12.5,v*100/4095-50,k_p*500/4095,k_d*5/4095,torque*50/4095-25,
                             motor_id=motor_id,channel=channel,feedback=feedback,quantized=True)
    return bytes([p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (k_p >> 8),
                  k_p & 255, k_d >> 4, ((k_d & 15) << 4) | (torque >> 8), torque & 255])

@dataclass(frozen=True)
class Feedback:
    measurement: MotorFeedback
    raw_count: int
    bus_timestamp: float
    temperature: int | None
    fault: int | None

def decode(message, motor_id, reply_id, channel, received_at):
    """Accept only external, standard, classic MIT feedback with matching IDs."""
    if (not message.is_rx or message.is_error_frame or message.is_remote_frame
        or message.is_extended_id or message.is_fd or message.arbitration_id!=reply_id
        or message.dlc not in (6,8) or len(message.data)!=message.dlc):
        return None
    d=message.data
    if d[0]!=motor_id:return None
    raw=(d[1]<<8)|d[2]
    measurement=MotorFeedback(motor_id,raw*25/65535-12.5,
        ((d[3]<<4)|(d[4]>>4))*100/4095-50, received_at,channel,
        (((d[4]&15)<<8)|d[5])*50/4095-25)
    return Feedback(measurement,raw,message.timestamp,d[6] if len(d)==8 else None,
                    d[7] if len(d)==8 else None)
