"""Bounded handoff from the sole socket reader to the controller worker.

The reader never acquires the controller lock or runs decoding/filtering/logging.
An empty-socket checkpoint is published AFTER all preceding frames are enqueued.
Taking messages and that checkpoint under the same short lock prevents timeout
decisions from overtaking a confirmation already captured by the reader.
"""
from collections import deque
from dataclasses import dataclass
import threading
import time

from .transport import TxConfirmation


@dataclass(frozen=True)
class Received:
    message: object
    monotonic: float
    wall_time: float


class ReceiveInbox:
    CAPACITY = 4096
    FEEDBACK_BATCH = 64

    def __init__(self):
        self._lock=threading.Lock()
        self._priority=deque();self._feedback=deque()
        self._empty_through=0.;self._error=None

    def put(self,msg):
        row=Received(msg,time.monotonic(),time.time())
        priority=(isinstance(msg,TxConfirmation) or msg.is_error_frame or not msg.is_rx
                  or (len(msg.data)==8 and msg.data[7]!=0))
        with self._lock:
            if len(self._priority)+len(self._feedback)>=self.CAPACITY:
                self._error='RX application inbox overflow; feedback history lost'
                return False
            (self._priority if priority else self._feedback).append(row)
        return True

    def empty_at(self,before_read):
        with self._lock:self._empty_through=before_read

    def fail(self,reason):
        with self._lock:
            if self._error is None:self._error=reason

    def take(self):
        with self._lock:
            priority=list(self._priority);self._priority.clear()
            feedback=[self._feedback.popleft() for _ in range(min(self.FEEDBACK_BATCH,len(self._feedback)))]
            return priority,feedback,self._empty_through,self._error,len(self._feedback)
