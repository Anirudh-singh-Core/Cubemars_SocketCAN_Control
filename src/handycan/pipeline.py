"""Worker-owned, nonblocking MIT exchanges. No socket, thread, or motor discovery.

One outstanding TX per ID; normal control does not wait for feedback. Startup
is serialized across IDs. Special commands need TX confirmation, not a fabricated
motor ACK. Their bounded response window keeps late special-command feedback out
of the subsequent probe. MIT has no transaction counter: timing is not an ACK.
"""
from dataclasses import dataclass
import errno
import time

from .protocol import POLL, ENABLE, DISABLE, SET_ZERO
from .errors import SessionFault
from .config import TimingPolicy


@dataclass
class Exchange:
    payload: bytes
    kind: str
    created: float
    response_timeout: float
    optional: bool = False
    submitted: float | None = None
    submitted_wall: float = 0.
    confirmed: float | None = None
    confirmed_wall: float = 0.
    feedback: float | None = None
    feedback_wall: float = 0.


class Pipeline:
    def __init__(self, ids, period, send, event, *, timing=None, clock=time.monotonic, wall=time.time,
                 rx_checked=None):
        self.timing=timing or TimingPolicy()
        self.period=period;self.send=send;self.event=event;self.clock=clock;self.wall=wall
        self.links={i:dict(stage='disabled',exchange=None,next_poll=0.,future=None,
                           disable_futures=[],response_timeout=.1) for i in ids}
        self.startup_owner=None
        self.rx_checked=rx_checked

    def _finish_future(self, future, error=None, **result):
        if future is not None and not future.done():
            if error:future.set_exception(error)
            else:future.set_result(result)

    def start(self, i, future, response_timeout):
        a=self.links[i]
        a.update(stage='queued',future=future,response_timeout=response_timeout)

    def zero(self,i,future):
        a=self.links[i];a.update(stage='zero',future=future)
        self._begin(i,SET_ZERO,'set_zero',self.timing.special_response_window_s,True)

    def cancel_zero(self,i):
        a=self.links[i]
        self._finish_future(a['future'],SessionFault('Set Zero interrupted by stop'))
        a['future']=None
        if a['exchange'] is None or a['exchange'].submitted is None:
            a.update(exchange=None,stage='disabled');self.event('zeroed',i)

    def available(self,i):
        a=self.links[i]
        return a['stage']=='ready' and a['exchange'] is None and self.clock()>=a['next_poll']

    def cancel_unsent_control(self,i):
        a=self.links[i];e=a['exchange']
        if e and e.kind=='control' and e.submitted is None:a['exchange']=None

    def disable(self, i, future=None):
        a=self.links[i]
        self._finish_future(a['future'],SessionFault('Enable interrupted by disable'))
        a['future']=None
        if future is not None:a['disable_futures'].append(future)
        if a['stage'] not in ('draining','disable'):
            a['stage']='draining'
            # A packet which never entered the kernel can be cancelled safely.
            if a['exchange'] and a['exchange'].submitted is None:a['exchange']=None
            elif a['exchange']:
                a['exchange'].optional=True
                a['exchange'].response_timeout=min(a['exchange'].response_timeout,self.timing.special_response_window_s)
        if self.startup_owner==i:self.startup_owner=None

    def control(self, i, payload):
        a=self.links[i]
        if not self.available(i):return False
        # Submit this tick's target once. TX-full drops this unsent sample;
        # the next tick generates a fresh target. No aged packet, no 2T fault.
        self._begin(i,payload,'control',0.)
        self._service(i)
        a['next_poll']=0.
        return a['exchange'] is not None

    def _begin(self, i, payload, kind, timeout, optional=False):
        self.links[i]['exchange']=Exchange(payload,kind,self.clock(),timeout,optional)

    def confirm(self, i, payload, timestamp, *, captured_monotonic=None,captured_wall=None):
        a=self.links.get(i);e=a['exchange'] if a else None
        if e is None or e.submitted is None or e.payload!=payload or timestamp<e.submitted_wall:
            self.event('unexpected_tx_confirmation',i,payload=payload.hex(),bus_timestamp=timestamp);return
        if e.confirmed is None:
            if timestamp-e.submitted_wall>self.timing.tx_confirmation_timeout_s:
                raise SessionFault(f'Motor {i}: late TX confirmation ({e.kind})')
            e.confirmed=self.clock()
            e.confirmed_wall=timestamp
            self.event('tx_confirmed',i,kind=e.kind,bus_timestamp=timestamp,
                       latency_s=e.confirmed-e.submitted,
                       kernel_tx_delay_s=timestamp-e.submitted_wall,
                       rx_capture_delay_s=None if captured_wall is None else max(0.,captured_wall-timestamp),
                       rx_dispatch_delay_s=None if captured_monotonic is None else max(0.,e.confirmed-captured_monotonic),
                       execution_verified=False)

    def feedback(self, i, timestamp):
        e=self.links[i]['exchange']
        if e and e.kind!='control' and e.submitted is not None and timestamp>=e.submitted_wall:
            e.feedback=self.clock();e.feedback_wall=timestamp

    def _service(self, i):
        a=self.links[i];e=a['exchange'];now=self.clock()
        if e is None:return True
        if e.submitted is None:
            if e.kind!='control' and now-e.created>=self.timing.special_tx_queue_timeout_s:
                raise SessionFault(f'Motor {i}: TX queue deadline ({e.kind})')
            wall=self.wall()
            try:self.send(i,e.payload,e.kind)
            except OSError as exc:
                if exc.errno in (errno.EAGAIN,errno.EWOULDBLOCK,errno.ENOBUFS):
                    if e.kind=='control':
                        a['exchange']=None
                        self.event('control_sample_dropped',i,reason='TX queue busy',errno=exc.errno)
                    return False
                raise
            e.submitted=now;e.submitted_wall=wall
        if e.confirmed is None:
            deadline=e.submitted+self.timing.tx_confirmation_timeout_s
            if now>=deadline:
                # A missing Python callback is not evidence of a missing kernel
                # echo. The reader must first drain past the deadline and this
                # worker must consume the priority inbox/checkpoint atomically.
                if self.rx_checked is not None and not self.rx_checked(deadline):
                    if now-deadline>max(self.timing.scheduler_stall_s,3*self.period):
                        raise SessionFault(f'Motor {i}: RX processing stalled; TX confirmation not yet adjudicated ({e.kind})')
                    return False
                raise SessionFault(f'Motor {i}: TX confirmation timeout after RX drain ({e.kind})')
            return False
        if e.kind=='control':
            a['exchange']=None
            return True
        if e.optional:
            # Replies to FC/FD/neutral may be absent. Observe the whole response
            # window even if one arrives; never turn silence into a motor ACK.
            complete=now-e.confirmed>=e.response_timeout
        else:
            complete=e.feedback is not None
            if complete and e.feedback_wall-e.confirmed_wall>e.response_timeout:
                raise SessionFault(f'Motor {i}: feedback arrived after response deadline ({e.kind})')
            if not complete and now-e.confirmed>=e.response_timeout:
                label='no initial feedback' if e.kind=='probe' else 'stale feedback'
                raise SessionFault(f'Motor {i}: {label}; response deadline ({e.kind})')
        if complete:a['exchange']=None
        return complete

    def advance(self):
        for i,a in self.links.items():
            try:complete=self._service(i)
            except Exception as exc:
                a['exchange']=None
                if a['stage']=='disable':
                    a['stage']='failed'
                    for f in a['disable_futures']:self._finish_future(f,exc)
                    a['disable_futures'].clear()
                    self.event('disable_failed',i,reason=str(exc))
                else:
                    if a['stage']=='zero':a['stage']='failed'
                    self._finish_future(a['future'],exc);a['future']=None
                    self.event('pipeline_fault',i,reason=str(exc))
                continue
            if not complete:continue
            stage=a['stage']
            if stage=='draining':
                a['stage']='disable'
                self._begin(i,DISABLE,'disable',self.timing.special_response_window_s,True)
            elif stage=='disable':
                a['stage']='disabled'
                self.event('disabled',i)
                for f in a['disable_futures']:
                    self._finish_future(f,submitted=True,transmission_confirmed=True,execution_verified=False)
                a['disable_futures'].clear()
            elif stage=='neutral':
                a['stage']='enable'
                self._begin(i,ENABLE,'enable',self.timing.special_response_window_s,True)
            elif stage=='enable':
                a['stage']='probe'
                self._begin(i,POLL,'probe',self.timing.startup_feedback_timeout_s)
            elif stage=='probe':
                a.update(stage='ready',next_poll=self.clock()+self.period)
                self.startup_owner=None
                self.event('ready',i)
                self._finish_future(a['future'],submitted=True,transmission_confirmed=True,
                                    feedback_verified=True,execution_verified=False)
                a['future']=None
            elif stage=='zero':
                a['stage']='disabled'
                self.event('zeroed',i)
                self._finish_future(a['future'],submitted=True,transmission_confirmed=True,
                                    execution_verified=False,limits_reference='new zero')
                a['future']=None
        # One startup at a time, without suspending already active motors.
        if self.startup_owner is None:
            for i,a in self.links.items():
                if a['stage']=='queued':
                    self.startup_owner=i;a['stage']='neutral'
                    self._begin(i,POLL,'neutral',self.timing.special_response_window_s,True)
                    break

    def pending(self):
        return any(a['stage'] in ('draining','disable','zero') for a in self.links.values())

    def finish_shutdown(self):
        error=SessionFault('Session closed before operation completed')
        for i,a in self.links.items():
            self._finish_future(a['future'],error);a['future']=None
            for f in a['disable_futures']:self._finish_future(f,error)
            a['disable_futures'].clear()
            if a['stage'] in ('draining','disable'):
                a['stage']='failed';self.event('disable_failed',i,reason=str(error))
            elif a['stage']=='zero':
                a['stage']='failed';self.event('pipeline_fault',i,reason=str(error))

    def wake_delay(self):
        now=self.clock();delay=self.timing.rx_poll_interval_s
        for a in self.links.values():
            e=a['exchange']
            if e is not None:
                if e.submitted is None:delay=min(delay,.0001)
                elif e.confirmed is None:
                    remaining=e.submitted+self.timing.tx_confirmation_timeout_s-now
                    # Do not spin/starve the reader while waiting for a drain
                    # checkpoint. Reception wakes the worker immediately.
                    if remaining>0:delay=min(delay,remaining)
                else:delay=min(delay,max(0.,e.confirmed+e.response_timeout-now))
            elif a['stage'] not in ('ready','disabled','failed'):return 0.
        return delay
