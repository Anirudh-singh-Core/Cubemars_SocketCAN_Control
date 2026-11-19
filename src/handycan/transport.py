"""One python-can Bus per session, guarded by a cooperative channel lease."""
import fcntl
import os
import errno
import select
import socket
import struct
from dataclasses import dataclass
from pathlib import Path
import can
from .errors import OwnershipError
from .interface import validate_channel

@dataclass(frozen=True)
class TxConfirmation:
    motor_id: int
    payload: bytes
    timestamp: float

# Linux UAPI values; timestamp layout matches the native Jetson/Pi ABI.
SO_TIMESTAMPNS=35
SO_RXQ_OVFL=40
TIMESPEC=struct.Struct('@ll')
CAN_FRAME=struct.Struct('=IB3x8s')

def decode_datagram(frame, ancillary, flags, channel):
    """Preserve MSG_CONFIRM, which python-can Message.is_rx alone cannot do."""
    if flags & (socket.MSG_TRUNC|socket.MSG_CTRUNC) or len(frame)!=16:
        raise can.CanOperationError('Truncated or non-classic CAN datagram')
    timestamp=None;drops=0
    for level,kind,data in ancillary:
        if level!=socket.SOL_SOCKET:continue
        if kind==SO_TIMESTAMPNS:
            sec,nsec=TIMESPEC.unpack_from(data)
            if not 0<=nsec<1000000000:raise can.CanOperationError('Invalid CAN timestamp')
            timestamp=sec+nsec*1e-9
        elif kind==SO_RXQ_OVFL:drops=struct.unpack_from('=I',data)[0]
    if drops:raise can.CanOperationError(f'SocketCAN RX queue overflow: {drops} frames lost')
    if timestamp is None:raise can.CanOperationError('Missing kernel CAN timestamp')
    ident,dlc,data=CAN_FRAME.unpack(frame)
    if dlc>8:raise can.CanOperationError('Invalid classic CAN DLC')
    extended=bool(ident&0x80000000);error=bool(ident&0x20000000)
    arb=ident & (0x1fffffff if extended or error else 0x7ff)
    if flags & socket.MSG_CONFIRM:
        if extended or error or ident&0x40000000:raise can.CanOperationError('Invalid TX confirmation')
        return TxConfirmation(arb,data[:dlc],timestamp)
    return can.Message(timestamp=timestamp,channel=channel,arbitration_id=arb,
        data=data[:dlc],dlc=dlc,is_extended_id=extended,is_error_frame=error,
        is_remote_frame=bool(ident&0x40000000),is_rx=not bool(flags&socket.MSG_DONTROUTE))

class ChannelLease:
    """Same-user lock, shared with the old beta lock path. Never delete live lock files."""
    def __init__(self,channel,folder=None):
        validate_channel(channel)
        self.path=(Path(folder) if folder else Path.home()/'.cache'/'handyflie')/(channel+'.lock')
        self.file=None
    def acquire(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        file=self.path.open('a+')
        try:fcntl.flock(file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as exc:
            file.close()
            raise OwnershipError(f'CAN channel is owned by another cooperating process: {self.path}') from exc
        self.file=file
        file.seek(0);file.truncate();file.write(f'pid={os.getpid()}\n');file.flush()
    def close(self):
        if self.file:self.file.close();self.file=None

class Transport:
    """One TX worker sends, one independent RX worker receives on the same bus."""
    def __init__(self,channel):
        self.channel=channel;self.lease=ChannelLease(channel);self.bus=None
    def open(self):
        self.lease.acquire()
        try:
            self.bus=can.Bus(interface='socketcan',channel=self.channel,
                             receive_own_messages=True,local_loopback=True,
                             ignore_config=True,fd=False)
            self._require_driver_echo()
            sock=self.bus.socket
            # python-can logs some option failures; require these options here.
            sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_RECV_OWN_MSGS,1)
            sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_LOOPBACK,1)
            sock.setsockopt(socket.SOL_CAN_RAW,2,0x1fffffff) # CAN_RAW_ERR_FILTER
            sock.setsockopt(socket.SOL_SOCKET,SO_TIMESTAMPNS,1)
            sock.setsockopt(socket.SOL_SOCKET,SO_RXQ_OVFL,1)
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024)
            sock.setblocking(False)
        except Exception:
            self.close();raise
    def _require_driver_echo(self):
        # Without IFF_ECHO Linux may provide a software echo before actual TX.
        flags=int((Path('/sys/class/net')/self.channel/'flags').read_text().strip(),16)
        if not flags & 0x40000:
            raise can.CanInitializationError('CAN driver must support IFF_ECHO for transmission confirmation')
    def send(self,motor_id,payload):
        try:
            self.bus.send(can.Message(arbitration_id=motor_id,data=payload,
                                      is_extended_id=False,check=True),timeout=0)
        except can.CanOperationError as exc:
            code=exc.error_code
            if code in (errno.EAGAIN,errno.EWOULDBLOCK,errno.ENOBUFS) or str(exc)=='Transmit buffer full':
                raise BlockingIOError(code or errno.EAGAIN,str(exc)) from exc
            raise
    def receive(self,timeout):
        # Same python-can socket, sole RX worker. No second bus/receiver.
        sock=self.bus.socket
        if not select.select([sock],[],[],timeout)[0]:return None
        try:
            frame,ancillary,flags,_=sock.recvmsg(72,socket.CMSG_SPACE(TIMESPEC.size)+socket.CMSG_SPACE(4))
        except BlockingIOError:return None
        return decode_datagram(frame,ancillary,flags,self.channel)
    def close(self):
        try:
            if self.bus:self.bus.shutdown()
        finally:
            self.bus=None;self.lease.close()
