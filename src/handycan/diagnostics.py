"""Bounded CAN tests through the existing Session; no extra socket or trajectory."""
import time
from dataclasses import asdict
from .errors import BusyError
from .interface import inspect_interface, FAULT_COUNTERS


def can_test(session, motor_id=None, *, active=False, window=1., _inspector=inspect_interface):
    """Test one explicit ID, or all configured IDs when motor_id is omitted.

    Active all-motor tests check IDs individually, then poll them together at the
    configured rate. They enable zero-gain monitoring and attempt disable on exit.
    Passive tests send nothing. PASS requires fresh feedback from EVERY tested ID
    and no new link errors; it does not certify motion, holding, or future health.
    """
    if not .05 <= window <= 2:raise ValueError('window must be .05..2 seconds')
    ids=list(session.axes) if motor_id is None else [motor_id]
    if not ids or any(isinstance(i,bool) or i not in session.axes for i in ids):
        raise ValueError('Unknown motor ID')
    if not session._operation.acquire(blocking=False):raise BusyError('Another exclusive operation is running')
    report=dict(result='FAIL',motor_id=motor_id,motor_ids=ids,motor_count=len(ids),
                channel=session.channel,active=active,control_hz=session.control_hz,
                timing=asdict(session.timing),communication_mode='sequential_startup_async_feedback',
                motor_models={i:session.axes[i]['config'].model for i in ids},
                phases=[],failure_reasons=[],execution_verified=False,
                communication_passed=False,interface_passed=False,link_quality_passed=False)
    enabled=set();stop_generation=session._stop_generation;initial=None

    def sample():
        return dict(time=time.monotonic(),interface=_inspector(session.channel),session=session.snapshot())

    def check_running():
        if session.fault:raise RuntimeError(session.fault)
        if session._closing.is_set() or session._stop.is_set() or session._stop_generation!=stop_generation:
            raise RuntimeError('Session stopped')

    def wait(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            check_running();time.sleep(min(.005,max(0.,end-time.monotonic())))

    def phase(name, group, operation):
        before=sample();error=None
        try:operation()
        except Exception as exc:error=str(exc)
        after=sample()
        delta={k:after['interface']['counters'].get(k,0)-before['interface']['counters'].get(k,0) for k in FAULT_COUNTERS}
        per_motor={}
        with session._lock:
            for i in group:
                times=[t for t in session.axes[i]['feedback_times'] if before['time']<=t<=after['time']]
                edges=[before['time']]+times+[after['time']]
                per_motor[i]=dict(feedback_frames=len(times),
                    tx_submitted=after['session']['axes'][i]['tx_submitted']-before['session']['axes'][i]['tx_submitted'],
                    tx_confirmed=after['session']['axes'][i]['tx_confirmed']-before['session']['axes'][i]['tx_confirmed'],
                    control_submitted=after['session']['axes'][i]['control_submitted']-before['session']['axes'][i]['control_submitted'],
                    feedback_hz=len(times)/max(after['time']-before['time'],1e-9),
                    max_feedback_gap_s=max(b-a for a,b in zip(edges,edges[1:])))
        row=dict(name=name,motor_ids=group,start=before['time'],end=after['time'],error=error,
                 motors=per_motor,feedback_frames=sum(m['feedback_frames'] for m in per_motor.values()),
                 max_feedback_gap_s=max(m['max_feedback_gap_s'] for m in per_motor.values()),
                 counter_delta=delta,initial_state=before['interface']['state'],final_state=after['interface']['state'],
                 interface_ready=before['interface']['ready'] and after['interface']['ready'],
                 interface_reasons=before['interface']['reasons']+after['interface']['reasons'],
                 error_frame_delta=after['session']['metrics']['error_frames']-before['session']['metrics']['error_frames'],
                 other_local_frames=after['session']['metrics']['other_local_frames']-before['session']['metrics']['other_local_frames'])
        report['phases'].append(row);session.journal.record('can_test_phase',**row)
        if active and row['other_local_frames']:report['failure_reasons'].append(f'{name}: competing local transmitter')
        if error:report['failure_reasons'].append(f'{name}: {error}')
        if not row['interface_ready']:report['failure_reasons'].append(f'{name}: interface not ready: {row["interface_reasons"]}')
        if any(delta.values()) or row['error_frame_delta']:
            report['failure_reasons'].append(f'{name}: new error/drop/restart counters {delta}; error frames {row["error_frame_delta"]}')
        return row

    def enable(group):
        for i in group:
            check_running();enabled.add(i)
            session._request('enable',motor_id=i)
        check_running()
        if not all(session.snapshot()['axes'][i]['state']=='MONITORING' for i in group):
            raise RuntimeError('No initial feedback from all requested motors')

    def disable(group):
        errors=[]
        for i in group:
            if i not in enabled:continue
            axis=session.snapshot()['axes'][i]
            if axis['state']=='DISABLED' and axis['disable_submitted']:
                enabled.discard(i);continue
            try:session._request('disable',motor_id=i);enabled.discard(i)
            except Exception as exc:errors.append(f'Motor {i}: {exc}')
        if errors:raise RuntimeError('; '.join(errors))

    def communication(row):
        passed=True
        for i,m in row['motors'].items():
            if m['feedback_frames']<2 or m['max_feedback_gap_s']>session.timing.diagnostic_feedback_gap_s:
                report['failure_reasons'].append(f'{row["name"]}: motor {i}: no sustained matching feedback')
                passed=False
        return passed

    try:
        if any(a['state']!='DISABLED' for a in session.snapshot()['axes'].values()):
            raise BusyError('Disable all motors before CAN Test')
        with session._lock:session._diagnostic_active=active
        check_running();initial=sample()
        session.journal.record('can_test_start',motor_ids=ids,active=active)
        baseline=phase('baseline',ids,lambda:wait(window))
        if not active:report['communication_passed']=communication(baseline)
        else:
            groups=[[i] for i in ids]+([ids] if len(ids)>1 else [])
            completed=0;all_communication=True
            for group in groups:
                if report['failure_reasons']:break
                prefix='' if len(ids)==1 else ('combined/' if len(group)>1 else f'motor_{group[0]}/')
                startup=phase(prefix+'enable',group,lambda:enable(group))
                if not startup['error'] and not report['failure_reasons']:
                    observed=phase(prefix+'polling',group,lambda:wait(window))
                    all_communication=communication(observed) and all_communication
                    completed+=1
                phase(prefix+'disable',group,lambda:disable(group))
                phase(prefix+'after_disable',group,lambda:wait(window))
            report['communication_passed']=all_communication and completed==len(groups)
            if not report['communication_passed']:report['failure_reasons'].append('Not all requested motor/group communication checks passed')
        final=sample()
        # Include errors in the gaps between phases, not just within each phase.
        delta={k:final['interface']['counters'].get(k,0)-initial['interface']['counters'].get(k,0) for k in FAULT_COUNTERS}
        report['counter_delta']=delta
        report['error_frame_delta']=final['session']['metrics']['error_frames']-initial['session']['metrics']['error_frames']
        report['interface_passed']=initial['interface']['ready'] and final['interface']['ready'] and all(p['interface_ready'] for p in report['phases'])
        report['link_quality_passed']=not any(delta.values()) and not report['error_frame_delta'] and all(not any(p['counter_delta'].values()) and not p['error_frame_delta'] for p in report['phases'])
        if not report['interface_passed']:report['failure_reasons'].append('Interface not healthy throughout test')
        if not report['link_quality_passed']:report['failure_reasons'].append('New CAN errors during test')
        check_running()
        report['result']='FAIL' if report['failure_reasons'] else 'PASS'
    except Exception as exc:report['failure_reasons'].append(str(exc))
    finally:
        try:
            if enabled:
                try:disable(list(enabled))
                except Exception as exc:report['failure_reasons'].append(f'Disable submission failed: {exc}')
            if report['failure_reasons']:report['result']='FAIL'
            if session.journal:session.journal.record('can_test_result',report=report)
        finally:
            with session._lock:session._diagnostic_active=False
            session._operation.release()
    return report
