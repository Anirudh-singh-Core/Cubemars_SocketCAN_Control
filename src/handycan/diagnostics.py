"""Phase-resolved CAN communication test using the caller's existing Session."""
import time
from .errors import BusyError
from .interface import inspect_interface,FAULT_COUNTERS

def can_test(session,motor_id,*,active=False,window=1.,_inspector=inspect_interface):
    """No motion targets. active=True explicitly enables neutral polling then disables.

    Passive mode sends nothing and requires existing external feedback. Diagnostics
    never create a Bus or socket. The returned report separates readiness,
    communication, and link quality; the CLI prints only Pass/Fail.
    """
    if not .05<=window<=2:raise ValueError('window must be .05..2 seconds')
    if motor_id not in session.axes:raise ValueError('Unknown motor ID')
    if not session._operation.acquire(blocking=False):raise BusyError('Another exclusive operation is running')
    report=dict(result='FAIL',motor_id=motor_id,channel=session.channel,active=active,
                phases=[],failure_reasons=[],execution_verified=False)
    enabled=False
    stop_generation=session._stop_generation
    def sample():
        status=_inspector(session.channel)
        return dict(time=time.monotonic(),interface=status,session=session.snapshot())
    def phase(name,operation):
        before=sample();error=None
        try:operation()
        except Exception as exc:error=str(exc)
        after=sample()
        delta={k:after['interface']['counters'].get(k,0)-before['interface']['counters'].get(k,0)
               for k in FAULT_COUNTERS}
        with session._lock:
            times=[t for t in session.axes[motor_id]['feedback_times'] if before['time']<=t<=after['time']]
        edges=[before['time']]+times+[after['time']]
        row=dict(name=name,start=before['time'],end=after['time'],error=error,
                 feedback_frames=len(times),max_feedback_gap_s=max(b-a for a,b in zip(edges,edges[1:])),
                 counter_delta=delta,initial_state=before['interface']['state'],final_state=after['interface']['state'],
                 interface_ready=before['interface']['ready'] and after['interface']['ready'],
                 interface_reasons=before['interface']['reasons']+after['interface']['reasons'],
                 error_frame_delta=after['session']['metrics']['error_frames']-before['session']['metrics']['error_frames'],
                 other_local_frames=after['session']['metrics']['other_local_frames']-before['session']['metrics']['other_local_frames'])
        report['phases'].append(row);session.journal.record('can_test_phase',motor_id=motor_id,**row)
        if active and row['other_local_frames']:
            report['failure_reasons'].append(f'{name}: competing local transmitter')
        if error:report['failure_reasons'].append(f'{name}: {error}')
        if not row['interface_ready']:report['failure_reasons'].append(f'{name}: interface not ready: {row["interface_reasons"]}')
        if any(delta.values()) or row['error_frame_delta']:
            report['failure_reasons'].append(f'{name}: new error/drop/restart counters {delta}; error frames {row["error_frame_delta"]}')
        return row
    def wait(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if session.fault:raise RuntimeError(session.fault)
            if session._closing.is_set() or session._stop.is_set() or session._stop_generation!=stop_generation:raise RuntimeError('Session stopped')
            time.sleep(min(.005,max(0.,end-time.monotonic())))
    def enable():
        nonlocal enabled
        enabled=True
        if session._stop_generation!=stop_generation:raise RuntimeError('Session stopped')
        session._request('enable',motor_id=motor_id)
        end=time.monotonic()+1
        while time.monotonic()<end:
            if session.fault:raise RuntimeError(session.fault)
            if session._stop_generation!=stop_generation:raise RuntimeError('Session stopped')
            if session.snapshot()['axes'][motor_id]['state']=='MONITORING':return
            time.sleep(.002)
        raise RuntimeError('No initial motor feedback')
    try:
        if any(a['state'] in ('ENABLING','MONITORING','MOVING','HOLDING','STREAMING') for a in session.snapshot()['axes'].values()):
            raise BusyError('Disable all motors before CAN Test')
        session.journal.record('can_test_start',motor_id=motor_id,active=active)
        baseline=phase('baseline',lambda:wait(window))
        observed=baseline
        if active and baseline['interface_ready'] and not baseline['error'] and not baseline['other_local_frames']:
            startup=phase('enable',enable)
            if not startup['error']:observed=phase('polling',lambda:wait(window))
        if enabled:
            phase('disable',lambda:session._request('disable',motor_id=motor_id))
            phase('after_disable',lambda:wait(window))
        observed_name='polling' if active else 'baseline'
        communication=(observed['name']==observed_name and observed['feedback_frames']>=2
                       and observed['max_feedback_gap_s']<=.1)
        report['communication_passed']=communication
        if not communication:report['failure_reasons'].append('No sustained matching feedback in observation phase')
        report['interface_passed']=all(p['interface_ready'] for p in report['phases'])
        report['link_quality_passed']=all(not any(p['counter_delta'].values()) and not p['error_frame_delta'] for p in report['phases'])
        report['result']='FAIL' if report['failure_reasons'] else 'PASS'
    except Exception as exc:report['failure_reasons'].append(str(exc))
    finally:
        if enabled and not any(p['name']=='disable' for p in report['phases']):
            try:session._request('disable',motor_id=motor_id)
            except Exception as exc:report['failure_reasons'].append(f'Disable submission failed: {exc}')
        if report['failure_reasons']:report['result']='FAIL'
        if session.journal:session.journal.record('can_test_result',report=report)
        session._operation.release()
    return report
