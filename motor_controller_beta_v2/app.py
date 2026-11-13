"""Thin GUI adapter: all motor I/O uses handycan.Session; no SocketCAN access here."""
from copy import deepcopy
from pathlib import Path
import json
import threading
import time
import uuid
from handycan import Motor, Session, can_test
from handycan.config import validate
from handycan.interface import inspect_interface

ROOT=Path(__file__).resolve().parents[1]

class Application:
    def __init__(self,session_factory=Session,diagnostic=can_test):
        self._factory=session_factory;self._diagnostic=diagnostic
        self.config=None;self.motors=();self.session=None;self.interface=None
        self.job=None;self._job_thread=None;self._guard=threading.RLock()
        self._actions=threading.Lock();self._closed=False

    def _running(self):return self._job_thread is not None and self._job_thread.is_alive()

    def snapshot(self):
        with self._guard:
            session=self.session;config=deepcopy(self.config);job=deepcopy(self.job)
            interface=deepcopy(self.interface)
        state=session.snapshot() if session else None
        readings={}
        if session and state and not state['fault']:
            for i,a in state['axes'].items():
                try:
                    reading=session.get_encoder_feedback(i)
                    if a['filter']!='raw':reading['filtered']=session.get_encoder_feedback(i,source=a['filter'])
                    readings[i]=reading
                except Exception:pass
        return dict(version='beta V2',backend='python-can / handycan',initialized=config is not None,
                    connected=bool(state and state['open']),config=config,session=state,
                    readings=readings,job=job,interface=interface)

    def stop(self):
        # Does not wait for the GUI action mutex or diagnostic completion.
        session=self.session
        if session:session.stop()

    def action(self,name,p):
        if not self._actions.acquire(blocking=False):raise RuntimeError('Another action is pending; Stop All remains available')
        try:
            if self._closed:raise RuntimeError('Server is shutting down')
            if name=='inspect':
                channel=self.config['channel'] if self.config else p.get('channel','can0')
                info=inspect_interface(channel)
                with self._guard:self.interface=info
                return info
            if self._running():raise RuntimeError('Diagnostic running; use Stop All to interrupt')
            if name=='configure':
                if self.session:raise RuntimeError('Disconnect before changing configuration')
                data=p['config'];channel=data.get('channel','can0');hz=data.get('control_hz',1000)
                motors=tuple(Motor.from_degrees(channel=channel,**row) for row in data['motors'])
                validate(motors,channel,hz)
                with self._guard:
                    self.config=dict(channel=channel,control_hz=hz,motors=deepcopy(data['motors']))
                    self.motors=motors;self.job=None
                return dict(initialized=True)
            if name=='connect':
                if self.config is None:raise RuntimeError('Initialize first')
                if self.session:raise RuntimeError('Already connected')
                path=ROOT/'diagnostic_logs'/'beta_V2'/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.jsonl')
                session=self._factory(self.motors,channel=self.config['channel'],
                                      control_hz=self.config['control_hz'],log_path=path)
                session.open()
                with self._guard:self.session=session;self.job=None
                return dict(connected=True,motor_commands_sent=False)
            if name=='disconnect':
                session=self.session
                if session:session.close()
                with self._guard:self.session=None
                return dict(connected=False)
            if not self.session:raise RuntimeError('Connect first')
            i=p.get('motor_id')
            if isinstance(i,bool) or not isinstance(i,int) or i not in self.session.axes:raise ValueError('Select a configured motor ID')
            if name=='test':
                if p.get('confirm') is not True:raise ValueError('Confirm active test: it enables, polls and disables')
                if any(a['state']!='DISABLED' for a in self.session.snapshot()['axes'].values()):
                    raise RuntimeError('Disable all motors before CAN Test')
                with self._guard:self.job=dict(state='RUNNING',motor_id=i,report=None,started=time.time())
                self._job_thread=threading.Thread(target=self._run_test,args=(self.session,i),name='gui-can-test',daemon=True)
                self._job_thread.start()
                return dict(started=True)
            if name=='enable':
                if p.get('confirm') is not True:raise ValueError('Confirm motor activation')
                return self.session.enable(i)
            if name=='disable':return self.session.disable(i)
            if name=='zero':return self.session.set_zero(i,confirm=p.get('confirm',False))
            if name in ('move','go_zero'):
                if p.get('confirm') is not True:raise ValueError('Confirm motion request')
                if name=='go_zero':return self.session.go_zero(i,p['duration'],kp=p['kp'],kd=p['kd'])
                return self.session.move(i,p['target_deg'],p['duration'],kp=p['kp'],kd=p['kd'],ff_torque=p.get('ff_torque',0))
            if name=='filter':
                return self.session.set_filter(i,p['source'],rest_variance_counts_squared=p.get('variance'),**p.get('parameters',{}))
            raise ValueError('Unknown action')
        finally:self._actions.release()

    def _run_test(self,session,motor_id):
        try:report=self._diagnostic(session,motor_id,active=True)
        except Exception as exc:report=dict(result='FAIL',failure_reasons=[str(exc)],phases=[])
        with self._guard:
            self.job.update(state=report['result'],report=report,finished=time.time())
        try:
            session.log_path.with_suffix('.can-test.json').write_text(json.dumps(report,indent=2)+'\n')
        except OSError as exc:
            with self._guard:self.job['save_error']=str(exc)

    def close(self):
        self._closed=True;self.stop()
        with self._actions:
            if self.session:self.session.close()
            if self._job_thread:self._job_thread.join(timeout=3)
            with self._guard:self.session=None
