"""One python-can Bus per session, guarded by a cooperative channel lease."""
import fcntl
import os
from pathlib import Path
import can
from .errors import OwnershipError
from .interface import validate_channel

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
    """Only Session's I/O worker calls send/receive after open."""
    def __init__(self,channel):
        self.channel=channel;self.lease=ChannelLease(channel);self.bus=None
    def open(self):
        self.lease.acquire()
        try:
            self.bus=can.Bus(interface='socketcan',channel=self.channel,
                             receive_own_messages=False,local_loopback=True,
                             ignore_config=True,fd=False)
        except Exception:
            self.close();raise
    def send(self,motor_id,payload):
        self.bus.send(can.Message(arbitration_id=motor_id,data=payload,
                                  is_extended_id=False,check=True),timeout=0)
    def receive(self,timeout):return self.bus.recv(timeout)
    def close(self):
        try:
            if self.bus:self.bus.shutdown()
        finally:
            self.bus=None;self.lease.close()
