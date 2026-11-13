"""One channel owner, one receiver, one transmission scheduler. No auto setup/resume."""
from collections import deque
from dataclasses import replace
from concurrent.futures import Future, TimeoutError
from pathlib import Path
import math
import queue
import threading
import time
import uuid
import can
from .config import validate
from .errors import BusyError, FeedbackError, SessionFault
from .filters import LowPass, AdaptiveEMA, Kalman1D
from .interface import require_ready
from .journal import Journal
from .motion import Trajectory
from .streaming import JointTarget
from .protocol import ENABLE, DISABLE, SET_ZERO, POLL, decode, encode_mit_command
from .transport import Transport

ACTIVE={'ENABLING','MONITORING','MOVING','HOLDING','STREAMING'}

class Session:
    """Reusable hardware session. open() is read-only; enable() is explicit.

    All motor commands pass through one worker. One operation lock arbitrates
    diagnostics vs public commands; all socket reads occur in that worker.
    Feedback used for limits/motion is always raw. get_encoder_feedback selects
    raw or calibrated filtered output for higher-level callers.
    """
    def __init__(self,motors,*,channel='can0',control_hz=1000,log_path=None,
                 _transport_factory=Transport,_inspector=require_ready):
        self.motors=tuple(motors);validate(self.motors,channel,control_hz)
        self.channel=channel;self.control_hz=control_hz
        self._factory=_transport_factory;self._inspector=_inspector
        self._transport=None;self._worker=None;self._health=None
        self._closing=threading.Event();self._stop=threading.Event()
        self._lock=threading.RLock();self._operation=threading.Lock()
        self._requests=queue.Queue(32);self._health_fault=None;self._stop_generation=0
        self.fault=None;self._closed=False;self._next=0.;self._last_tx=None
        self._metrics=dict(cycles=0,missed_deadlines=0,max_lateness_s=0.,
                           max_tx_interval_s=0.,rx_frames=0,error_frames=0,other_local_frames=0)
        self.axes={m.limits.motor_id:dict(config=m,state='DISABLED',feedback=None,
            started=0.,feedback_after=0.,path=None,disable_submitted=False,filter=None,filter_name='raw',
            filtered=None,rest_variance=None,frames=0,position_warning=False,
            feedback_times=deque(maxlen=20000)) for m in self.motors}
        self.log_path=Path(log_path) if log_path else Path('logs')/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.jsonl')
        self.journal=None
        self._stream=None

    def open(self):
        if self._closed:raise SessionFault('Session closed; create a new session')
        if self._transport is not None:raise BusyError('Session already open')
        self._inspector(self.channel)
        self.journal=Journal(self.log_path)
        transport=self._factory(self.channel)
        try:transport.open()
        except Exception:
            self.journal.close();raise
        self._transport=transport
        self._worker=threading.Thread(target=self._run,name='can-session',daemon=True)
        self._health=threading.Thread(target=self._health_loop,name='can-health',daemon=True)
        self._worker.start();self._health.start()
        self.journal.record('session_open',channel=self.channel,control_hz=self.control_hz,
                            motor_ids=list(self.axes))
        return self
    def __enter__(self):return self.open()
    def __exit__(self,*args):self.close()

    def _health_loop(self):
        while not self._closing.wait(.5):
            try:self._inspector(self.channel)
            except Exception as exc:
                with self._lock:self._health_fault=f'Interface health: {exc}'
                return

    def _request(self,action,**data):
        if not self._worker or not self._worker.is_alive() or self._closing.is_set():
            raise SessionFault('Session is not open')
        future=Future()
        try:self._requests.put_nowait((action,data,future))
        except queue.Full:raise BusyError('Command queue is full') from None
        try:return future.result(timeout=2)
        except TimeoutError:
            future.cancel();self._stop.set()
            raise SessionFault('Command deadline expired; stop requested') from None

    def _public(self,action,**data):
        if not self._operation.acquire(blocking=False):raise BusyError('Exclusive operation in progress')
        try:return self._request(action,**data)
        finally:self._operation.release()

    def enable(self,motor_id):return self._public('enable',motor_id=motor_id)
    def wait_for_feedback(self,motor_id,timeout=1.):
        """Wait for fresh monitored feedback without sending extra frames."""
        if not math.isfinite(timeout) or timeout<=0:raise ValueError('Positive finite timeout required')
        end=time.monotonic()+timeout
        while True:
            try:return self.get_encoder_feedback(motor_id)
            except FeedbackError:
                if time.monotonic()>=end:raise FeedbackError('Timed out waiting for feedback') from None
                time.sleep(min(.002,max(0.,end-time.monotonic())))

    def disable(self,motor_id):return self._public('disable',motor_id=motor_id)
    def set_zero(self,motor_id,*,confirm=False):return self._public('zero',motor_id=motor_id,confirm=confirm)
    def move(self,motor_id,target_deg,duration,*,kp,kd,ff_torque=0):
        return self._public('move',motor_id=motor_id,target_deg=target_deg,duration=duration,kp=kp,kd=kd,ff_torque=ff_torque)
    def go_zero(self,motor_id,duration,*,kp,kd):
        return self._public('go_zero',motor_id=motor_id,target_deg=0,duration=duration,kp=kp,kd=kd,ff_torque=0)
    def start_stream(self,targets,*,timeout,max_step_deg,timeout_action):
        """Begin one group on already monitored motors; returns a stream token.

        targets maps motor IDs to JointTarget. timeout_action is explicitly
        'disable' (fault session) or 'hold' (hold last applied positions).
        No automatic enable. The caller must supply a smooth trajectory.
        """
        return self._public('stream_start',targets=dict(targets),timeout=timeout,
                            max_step_deg=max_step_deg,timeout_action=timeout_action)

    def stream_targets(self,token,targets,*,sequence):
        """Replace the pending group atomically, without a FIFO or CAN writes.

        Sequence must strictly increase. Acceptance is not transmission or
        execution confirmation. One producer should own the returned token.
        """
        targets=dict(targets)
        if not self._operation.acquire(blocking=False):raise BusyError('Exclusive operation in progress')
        try:
            with self._lock:
                stream=self._stream
                if self.fault or self._closing.is_set() or self._stop.is_set():raise SessionFault('Session stopped or faulted')
                if stream is None or token!=stream['token']:raise BusyError('Stream token is no longer active')
                now=time.monotonic()
                if now-stream['received']>=stream['timeout']:raise SessionFault('Stream command deadline expired')
                if isinstance(sequence,bool) or not isinstance(sequence,int) or sequence<=stream['sequence']:
                    raise ValueError('Sequence must be an integer greater than the last accepted sequence')
                if set(targets)!=set(stream['targets']):raise ValueError('Supply exactly the complete streaming group')
                reference=stream['applied']
                self._validate_targets(targets,reference,stream['max_step_deg'],now-stream['applied_at'])
                replaced=stream['pending'] is not None
                stream.update(targets=targets,pending=targets,received=now,sequence=sequence)
                if replaced:stream['replaced']+=1
                self.journal.record('stream_update',token=token,sequence=sequence,
                    targets={i:vars(t) for i,t in targets.items()},replaced_pending=replaced)
                return dict(accepted=True,sequence=sequence,execution_verified=False)
        finally:self._operation.release()

    def end_stream(self,token,*,action):
        """Explicitly 'hold' last applied positions or 'disable' the whole group."""
        return self._public('stream_end',token=token,end_action=action)

    def _validate_targets(self,targets,reference,max_step_deg,elapsed):
        if not targets:raise ValueError('Provide at least one joint target')
        for i,t in targets.items():
            if isinstance(i,bool) or not isinstance(i,int) or i not in self.axes or not isinstance(t,JointTarget):
                raise ValueError('Targets must map configured integer IDs to JointTarget values')
            a=self.axes[i];lim=a['config'].limits
            if a['state'] not in ('MONITORING','HOLDING','STREAMING') or a['feedback'] is None:
                raise BusyError('All group motors must be monitored with fresh feedback')
            p,v=t.radians()
            encode_mit_command(p,v,t.kp,t.kd,t.ff_torque,limits=lim,
                feedback=a['feedback'].measurement,motor_id=i,channel=self.channel)
            previous=reference[i].radians()[0] if reference else lim.trajectory_start(a['feedback'].measurement.position_rad)
            delta=p-previous
            if abs(math.degrees(delta))>max_step_deg:raise ValueError(f'Motor {i}: streaming position step exceeds max_step_deg')
            # Bound position slew as well as the supplied velocity field. Quantized
            # feedback at stream entry may differ by one transmitted position count.
            dt=max(1/self.control_hz,elapsed);tol=lim.position_tolerance_rad
            if not lim.velocity_min_rad_s*dt-tol<=delta<=lim.velocity_max_rad_s*dt+tol:
                raise ValueError(f'Motor {i}: streaming position slew exceeds velocity limits')
            if lim.position_period_rad and lim.position_max_rad-lim.position_min_rad<lim.position_period_rad:
                interval=lim.position_interval(previous,tol)
                if interval is None or not interval[0]<=p<=interval[1]:
                    raise ValueError(f'Motor {i}: streaming step crosses an excluded joint region')

    def _finish_stream(self,action,reason):
        stream=self._stream
        if stream is None:raise BusyError('No active stream')
        if action not in ('disable','hold'):raise ValueError('Choose hold or disable')
        targets=stream['applied']
        if action=='hold' and targets is None:raise BusyError('No applied sample to hold; use disable')
        if action=='hold':
            # Validate every zero-velocity hold before altering any member.
            for i,t in targets.items():
                a=self.axes[i];p,_=t.radians()
                encode_mit_command(p,0,t.kp,t.kd,t.ff_torque,limits=a['config'].limits,
                    feedback=a['feedback'].measurement,motor_id=i,channel=self.channel)
        self._stream=None
        self.journal.record('stream_end',token=stream['token'],action=action,reason=reason)
        if action=='disable':
            errors=[]
            for i in stream['targets']:
                try:self._disable(i)
                except Exception as exc:errors.append(str(exc))
            if errors:
                reason='Stream disable failed: '+'; '.join(errors)
                self._halt(reason)
                raise SessionFault(reason)
        else:
            now=time.monotonic()
            for i,t in targets.items():
                p,_=t.radians();self.axes[i].update(state='HOLDING',path=Trajectory(p,p,1,now,t.kp,t.kd,t.ff_torque))
        return dict(ended=True,action=action,execution_verified=False)

    def stop(self):
        """Priority stop; cancels pending commands and attempts FD on active motors."""
        with self._lock:self._stop_generation+=1
        self._stop.set()
    def set_filter(self,motor_id,source,*,rest_variance_counts_squared=None,**parameters):
        return self._public('filter',motor_id=motor_id,source=source,
                            variance=rest_variance_counts_squared,parameters=parameters)

    def snapshot(self):
        with self._lock:
            now=time.monotonic()
            return dict(channel=self.channel,open=self._transport is not None,fault=self.fault,
                metrics=dict(self._metrics),log_path=str(self.log_path),
                stream=None if self._stream is None else dict(token=self._stream['token'],
                    motor_ids=list(self._stream['targets']),sequence=self._stream['sequence'],
                    applied_sequence=self._stream['applied_sequence'],replaced_updates=self._stream['replaced'],
                    command_age_s=now-self._stream['received'],timeout=self._stream['timeout'],
                    timeout_action=self._stream['timeout_action']),
                logging=dict(dropped=self.journal.dropped,error=self.journal.error) if self.journal else None,
                axes={i:dict(state=a['state'],feedback_frames=a['frames'],
                    feedback_age_s=now-a['feedback'].measurement.timestamp if a['feedback'] else None,
                    disable_submitted=a['disable_submitted'],position_warning=a['position_warning'],
                    filter=a['filter_name']) for i,a in self.axes.items()})

    def get_encoder_feedback(self,motor_id,*,source='raw'):
        """Return fresh degree feedback; filtered angle never replaces raw safety data."""
        with self._lock:
            if self.fault:raise SessionFault(self.fault)
            a=self.axes[motor_id];fb=a['feedback']
            if fb is None:raise FeedbackError('No feedback; explicitly enable/monitor first')
            if time.monotonic()-fb.measurement.timestamp>a['config'].limits.feedback_max_age_s:
                raise FeedbackError('Feedback is stale')
            if source=='raw':count=fb.raw_count
            elif source==a['filter_name'] and a['filtered'] is not None:count=a['filtered']
            else:raise FeedbackError('Requested filter is not configured or has no fresh output')
            deg=math.degrees(count*25/65535-12.5)
            return dict(motor_id=motor_id,source=source,position_deg=deg,position_single_turn_deg=deg%360,
                raw_position_deg=math.degrees(fb.measurement.position_rad),raw_count=fb.raw_count,
                velocity_deg_s=math.degrees(fb.measurement.velocity_rad_s),torque_nm=fb.measurement.torque_nm,
                received_monotonic=fb.measurement.timestamp,bus_timestamp=fb.bus_timestamp,
                temperature=fb.temperature,fault_code=fb.fault,valid=True)

    def _send(self,motor_id,payload,kind):
        self._transport.send(motor_id,payload)
        self.journal.record('tx',motor_id=motor_id,kind=kind,payload=payload.hex(),execution_verified=False)

    def _disable(self,motor_id):
        a=self.axes[motor_id]
        try:
            self._send(motor_id,DISABLE,'disable');a['disable_submitted']=True
        except Exception:
            a['disable_submitted']=False;a['state']='FAULT';a['path']=None
            raise
        a['state']='DISABLED';a['path']=None
        return dict(submitted=True,execution_verified=False)

    def _halt(self,reason,fault=True):
        self._stream=None
        if fault:self.fault=reason
        self.journal.record('fault' if fault else 'stop',reason=reason)
        for motor_id,a in self.axes.items():
            if a['state'] in ACTIVE:
                try:self._disable(motor_id)
                except Exception as exc:self.journal.record('disable_failed',motor_id=motor_id,reason=str(exc))
        while True:
            try:_,_,future=self._requests.get_nowait()
            except queue.Empty:break
            if future.set_running_or_notify_cancel():future.set_exception(SessionFault(reason))

    def _execute(self,action,data):
        if action=='stream_start':
            if self.fault or self._stop.is_set():raise SessionFault('Session stopped or faulted')
            if self._stream is not None:raise BusyError('A stream is already active')
            timeout=data['timeout'];step=data['max_step_deg'];policy=data['timeout_action'];targets=data['targets']
            if any(isinstance(x,bool) or not isinstance(x,(float,int)) or not math.isfinite(x) or x<=0 for x in (timeout,step)):
                raise ValueError('Positive finite timeout and max_step_deg required')
            if timeout<2/self.control_hz or timeout>1:raise ValueError('Timeout must be between two control periods and 1 second')
            if policy not in ('hold','disable'):raise ValueError('Explicit timeout_action hold or disable required')
            self._validate_targets(targets,None,step,1/self.control_hz)
            now=time.monotonic();token=uuid.uuid4().hex
            self._stream=dict(token=token,targets=targets,pending=targets,applied=None,
                received=now,applied_at=now,sequence=0,applied_sequence=None,replaced=0,
                timeout=timeout,max_step_deg=step,timeout_action=policy)
            for i in targets:self.axes[i].update(state='STREAMING',path=None)
            self.journal.record('stream_start',token=token,timeout=timeout,max_step_deg=step,
                timeout_action=policy,targets={i:vars(t) for i,t in targets.items()})
            return token
        if action=='stream_end':
            if self._stream is None or data['token']!=self._stream['token']:raise BusyError('Stream token is no longer active')
            return self._finish_stream(data['end_action'],'Caller ended stream')
        i=data['motor_id']
        if isinstance(i,bool) or not isinstance(i,int) or i not in self.axes:raise ValueError('Unknown motor ID')
        a=self.axes[i];lim=a['config'].limits
        self.journal.record('request',action=action,arguments=data)
        if action=='disable':
            if self._stream and i in self._stream['targets']:return self._finish_stream('disable','Group member disabled')
            return self._disable(i)
        if self.fault:raise SessionFault(self.fault+'; close, resolve cause, and open a new session')
        if action=='enable':
            if a['state']!='DISABLED':raise BusyError('Motor must be disabled before enable')
            a.update(feedback=None,filtered=None,path=None,state='ENABLING',started=time.monotonic(),feedback_after=time.monotonic(),disable_submitted=False)
            a['feedback_times'].clear()
            self._reset_filter(a)
            # Mark active before sending so uncertain enable delivery triggers cleanup.
            self._send(i,POLL,'neutral');self._send(i,ENABLE,'enable')
            return dict(submitted=True,execution_verified=False)
        if action=='zero':
            if data['confirm'] is not True or a['state']!='DISABLED':raise ValueError('Set Zero requires disabled motor and confirm=True')
            a.update(feedback=None,filtered=None,filter=None,filter_name='raw',rest_variance=None,path=None,feedback_after=time.monotonic())
            self._send(i,SET_ZERO,'set_zero')
            return dict(submitted=True,execution_verified=False,limits_reference='new zero')
        if action=='filter':
            source=data['source'];params=data['parameters'];variance=data['variance']
            if source=='raw':f=None
            else:
                if variance is None or isinstance(variance,bool) or not math.isfinite(variance) or variance<0:
                    raise ValueError('Provide verified stationary variance in CAN counts squared')
                if source=='lowpass':f=LowPass(**params)
                elif source=='adaptive_ema':f=AdaptiveEMA(**params).use_rest_variance(variance)
                elif source=='kalman':f=Kalman1D(**params).use_rest_variance(variance)
                else:raise ValueError('Use raw, lowpass, adaptive_ema, or kalman')
            a.update(filter=f,filter_name=source,filtered=None,rest_variance=variance)
            return dict(source=source)
        if action in ('move','go_zero'):
            if a['state'] not in ('MONITORING','HOLDING'):raise BusyError('Enable and await fresh feedback; finish current motion first')
            target=data['target_deg'];duration=data['duration']
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in (target,duration)):
                raise ValueError('Target and duration must be finite')
            if not -360<=target<=360 or not .01<=duration<=3600:raise ValueError('Target -360..360 degrees; duration .01..3600 seconds')
            lim.check_feedback(a['feedback'].measurement)
            lim.check_gains_torque(data['kp'],data['kd'],data['ff_torque'])
            if data['kp']<=0:raise ValueError('Position control requires Kp > 0')
            recovery=action=='go_zero'
            if recovery:
                if lim.position_period_rad is not None:raise ValueError('Go Zero recovery requires signed linear limits')
                start=a['feedback'].measurement.position_rad
                motion_limits=replace(lim,position_min_rad=min(start,0),position_max_rad=max(start,0))
            else:
                start=lim.trajectory_start(a['feedback'].measurement.position_rad)
                motion_limits=lim
            target=math.radians(target)
            motion_limits.check_quintic(start,target,duration)
            a['path']=Trajectory(start,target,duration,time.monotonic(),data['kp'],data['kd'],data['ff_torque'],recovery)
            a['state']='MOVING'
            return dict(accepted=True,target_deg=data['target_deg'],hold_at_arrival=True)
        raise ValueError('Unknown operation')

    def _reset_filter(self,a):
        f=a['filter']
        if isinstance(f,LowPass):a['filter']=LowPass(f.cutoff)
        elif isinstance(f,AdaptiveEMA):a['filter']=AdaptiveEMA(f.min_hz,f.max_hz).use_rest_variance(a['rest_variance'])
        elif isinstance(f,Kalman1D):a['filter']=Kalman1D(f.q_per_second,r_floor=f.r_floor).use_rest_variance(a['rest_variance'])

    def _receive(self,msg):
        now=time.monotonic()
        if msg.is_error_frame:
            self._metrics['error_frames']+=1
            self.journal.record('can_error_frame',can_id=msg.arbitration_id,data=bytes(msg.data).hex(),bus_timestamp=msg.timestamp)
            # CAN_ERR_BUSOFF=0x40. Other transient errors are retained for diagnostics.
            if msg.arbitration_id & 0x40:raise SessionFault('CAN BUS-OFF error frame')
            return
        if not msg.is_rx:
            self._metrics['other_local_frames']+=1
            self.journal.record('other_local_tx',can_id=msg.arbitration_id)
            if any(a['state'] in ACTIVE for a in self.axes.values()):raise SessionFault('Competing local CAN transmitter')
            return
        for i,a in self.axes.items():
            age=max(0.,time.time()-msg.timestamp) if msg.timestamp else 0.
            fb=decode(msg,i,a['config'].reply_id,self.channel,now-age)
            if fb is None or fb.measurement.timestamp<a['feedback_after']:continue
            if fb.fault:raise FeedbackError(f'Motor {i}: driver fault byte 0x{fb.fault:02X}')
            if age>a['config'].limits.feedback_max_age_s:
                self.journal.record('stale_rx',motor_id=i,age=age);continue
            previous=a['feedback']
            if previous:
                if fb.measurement.timestamp<=previous.measurement.timestamp:continue
                if abs(fb.raw_count-previous.raw_count)>32767:raise FeedbackError(f'Motor {i}: encoder discontinuity')
            if a['state'] in ACTIVE:a['config'].limits.check_feedback(fb.measurement)
            a['feedback']=fb;a['frames']+=1;a['feedback_times'].append(now)
            a['position_warning']=a['config'].limits.position_interval(fb.measurement.position_rad,a['config'].limits.position_tolerance_rad) is None
            if a['filter']:a['filtered']=a['filter'].add(fb.measurement.timestamp,fb.raw_count)
            if a['state']=='ENABLING':a['state']='MONITORING'
            self._metrics['rx_frames']+=1
            self.journal.record('feedback',motor_id=i,raw_count=fb.raw_count,
                position_rad=fb.measurement.position_rad,velocity_rad_s=fb.measurement.velocity_rad_s,
                torque_nm=fb.measurement.torque_nm,filtered_count=a['filtered'],filter=a['filter_name'],
                bus_timestamp=msg.timestamp,received_monotonic=fb.measurement.timestamp,fault_code=fb.fault)

    def _tick(self,now):
        if self._stream and now-self._stream['received']>=self._stream['timeout']:
            if self._stream['timeout_action']=='disable':raise SessionFault('Streaming command deadline expired')
            self._finish_stream('hold','Streaming command deadline expired')
        active=[(i,a) for i,a in self.axes.items() if a['state'] in ACTIVE]
        if not active:self._next=now;self._last_tx=None;return
        for i,a in active:
            if a['state']=='ENABLING':
                if now-a['started']>1:raise FeedbackError(f'Motor {i}: no initial feedback within 1 second')
            elif not a['feedback'] or now-a['feedback'].measurement.timestamp>a['config'].limits.feedback_max_age_s:
                raise FeedbackError(f'Motor {i}: stale feedback')
        if now<self._next:return
        period=1/self.control_hz;late=now-self._next
        self._metrics['max_lateness_s']=max(late,self._metrics['max_lateness_s'])
        if late>max(.05,3*period):raise SessionFault('Control scheduler stalled')
        self._metrics['missed_deadlines']+=int(late/period)
        if self._last_tx is not None:
            self._metrics['max_tx_interval_s']=max(now-self._last_tx,self._metrics['max_tx_interval_s'])
        stream=self._stream
        commands=[]
        # Preflight the entire cycle before submitting any frames. CAN frames
        # still serialize on the wire; this is not hardware-synchronous motion.
        for i,a in active:
            if a['state'] in ('ENABLING','MONITORING'):payload=POLL
            elif a['state']=='STREAMING':
                t=(stream['pending'] or stream['applied'])[i];p,v=t.radians()
                payload=encode_mit_command(p,v,t.kp,t.kd,t.ff_torque,limits=a['config'].limits,
                    feedback=a['feedback'].measurement,motor_id=i,channel=self.channel)
            else:
                path=a['path'];p,v=path.sample(now)
                command_limits=a['config'].limits
                if path.zero_recovery:
                    # Only this immutable trajectory gets a corridor to zero;
                    # original configured limits and measured warnings stay intact.
                    command_limits=replace(command_limits,position_min_rad=min(path.start,0),position_max_rad=max(path.start,0))
                    if not min(path.start,0)<=p<=max(path.start,0) or v*path.start>0:
                        raise SessionFault('Go Zero recovery must progress toward zero')
                payload=encode_mit_command(p,v,path.kp,path.kd,path.torque,limits=command_limits,
                    feedback=a['feedback'].measurement,motor_id=i,channel=self.channel)
                if now-path.began>=path.duration:a['state']='HOLDING'
            commands.append((i,payload))
        for i,payload in commands:self._send(i,payload,'control')
        if stream and stream['pending'] is not None:
            stream.update(applied=stream['pending'],pending=None,applied_at=now,applied_sequence=stream['sequence'])
            self.journal.record('stream_applied',token=stream['token'],sequence=stream['sequence'],
                                cycle_time=now,execution_verified=False)
        self._metrics['cycles']+=1;self._last_tx=now
        # Absolute schedule; skip missed slots, never transmit catch-up bursts.
        self._next+=(int(late/period)+1)*period

    def _run(self):
        try:
            while not self._closing.is_set():
                with self._lock:
                    if self._health_fault:
                        self._halt(self._health_fault);self._health_fault=None
                    if self._stop.is_set():self._halt('Operator stop',fault=False);self._stop.clear()
                    for _ in range(4):
                        try:action,data,future=self._requests.get_nowait()
                        except queue.Empty:break
                        if not future.set_running_or_notify_cancel():continue
                        try:future.set_result(self._execute(action,data))
                        except Exception as exc:
                            if isinstance(exc,(can.CanError,OSError)):
                                self._halt(f'Transport failure: {exc}')
                            self.journal.record('request_failed',action=action,reason=str(exc))
                            future.set_exception(exc)
                    if not self.fault:
                        try:self._tick(time.monotonic())
                        except Exception as exc:self._halt(str(exc))
                    active=any(a['state'] in ACTIVE for a in self.axes.values())
                    timeout=max(0.,min(.005,self._next-time.monotonic())) if active else .005
                try:msg=self._transport.receive(timeout)
                except Exception as exc:
                    with self._lock:self._halt(f'Receive failure: {exc}')
                    self._closing.wait(.01);continue
                if msg is not None:
                    with self._lock:
                        try:self._receive(msg)
                        except Exception as exc:self._halt(str(exc))
        except Exception as exc:
            with self._lock:self._halt(f'Worker failure: {exc}')
        finally:
            with self._lock:self._halt('Session shutdown',fault=False)

    def close(self):
        if self._closed:return
        self._closing.set();self._stop.set()
        if self._worker:self._worker.join(timeout=3)
        if self._worker and self._worker.is_alive():raise SessionFault('Worker did not exit; transport ownership retained')
        if self._health:self._health.join(timeout=2)
        if self._transport:self._transport.close();self._transport=None
        if self.journal:self.journal.close()
        self._closed=True
