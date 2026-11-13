#!/usr/bin/env python3
"""Common live/static graphs and full diagnostic event history. No motor commands.

with graph_logger('my_run', metadata={'CAN_ID': 1}) as log:
    log.log('user_input', theta=90, duration_s=3)
    log.log('feedback', motor_id=1, raw_count=30000)
    log.plot_static([0,1], {'raw_deg':[0,1]}, ylabel='degrees')

For existing motor scripts record_event() uses one lazy process-wide session.
Live plots keep a 10-second window, but events.jsonl retains the full supplied
history. Display queue drops are counted and do not remove logged samples.
clear_up deletes logger sessions AND legacy encoder_records; refuses while any
integrated logger session is active. It does not remove code/configuration.
"""
import argparse
import atexit
from dataclasses import asdict, is_dataclass
from collections import deque
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import math
import multiprocessing as mp
from pathlib import Path
from queue import Empty, Full
import shutil
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parent
LOG_ROOT=ROOT/'diagnostic_logs'
_DEFAULT=None
_DEFAULT_LOCK=threading.Lock()
_WARNED=False
DEGREES_PER_COUNT=math.degrees(25/65535)


def _json(value):
    if is_dataclass(value) and not isinstance(value,type): return asdict(value)
    if isinstance(value,Path): return str(value)
    if hasattr(value,'item'): return value.item()
    if hasattr(value,'tolist'): return value.tolist()
    raise TypeError(f'Cannot serialize {type(value).__name__}')


class GraphLogger:
    def __init__(self, name='session', metadata=None):
        # This stable lock is outside cleared directories. Each writer holds SH.
        self._lease=(ROOT/'.graph_logger.lock').open('a+')
        fcntl.flock(self._lease,fcntl.LOCK_SH)
        self._lock=threading.Lock()
        self.closed=False
        self.started=time.monotonic()
        safe=''.join(c if c.isalnum() or c in '-_' else '_' for c in name)
        self.folder=LOG_ROOT/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+safe+'_'+uuid.uuid4().hex[:8])
        try:
            self.folder.mkdir(parents=True)
            self._file=(self.folder/'events.jsonl').open('w',buffering=65536)
            self._last_flush=time.monotonic()
            self.log('session_start',name=name,metadata=metadata or {},wall_time_utc=datetime.now(timezone.utc).isoformat())
        except BaseException:
            self._lease.close()
            raise

    def log(self,event,**fields):
        with self._lock:
            if self.closed: raise RuntimeError('Logger is closed')
            now=time.monotonic()
            item=dict(event=event,elapsed_s=now-self.started,monotonic_s=now,fields=fields)
            self._file.write(json.dumps(item,default=_json,allow_nan=False)+'\n')
            if now-self._last_flush>=1:
                self._file.flush();self._last_flush=now

    def plot_static(self,times,series,*,ylabel='value',title='Recorded history',show=False):
        # Full series persisted as well as rendered; no implicit downsampling.
        self.log('static_plot',times=list(times),series={k:list(v) for k,v in series.items()},ylabel=ylabel,title=title)
        return plot_static(times,series,self.folder/'history.png',ylabel=ylabel,title=title,show=show)

    def live_plot(self, kind='raw'):
        """Context yielding (sink, closed_event, process); sink.sample(...) appends.

        Example: with log.live_plot() as (sink, closed, process):
                     sink.sample(0.0, 30000, 0.0)
        GUI creation must run under a __main__ guard in spawned applications.
        """
        return plot_session(True,kind=kind,logger=self)

    def flush(self):
        """Flush buffered history before offline analysis or download."""
        with self._lock:
            if not self.closed: self._file.flush()

    def close(self):
        with self._lock:
            if not self.closed:
                try: self._file.close()
                finally:
                    self.closed=True;self._lease.close()

    def __enter__(self): return self
    def __exit__(self,*_): self.close()


def graph_logger(name='session', *, clear_up=False, metadata=None):
    """Create a logger, or explicitly clear all diagnostic/recorded data.

    Python syntax: graph_logger(clear_up=True). CLI: --clear-up or -clear_up.
    Clearing is never automatic and is not performed merely by importing.
    """
    if not clear_up: return GraphLogger(name,metadata)
    with (ROOT/'.graph_logger.lock').open('a+') as lock:
        try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('Stop/close active logger sessions before clearing logs')
        removed=[]
        for folder in (LOG_ROOT,ROOT/'encoder_records'):
            if folder.is_symlink(): raise RuntimeError(f'Refusing to clear symlink: {folder}')
            if folder.exists(): shutil.rmtree(folder);removed.append(str(folder))
        return dict(cleared=removed)


def default_logger():
    global _DEFAULT
    with _DEFAULT_LOCK:
        if _DEFAULT is None or _DEFAULT.closed:
            _DEFAULT=GraphLogger(Path(sys.argv[0]).stem or 'python')
        return _DEFAULT


def record_event(event, **fields):
    """Best-effort shared diagnostics; failures never prevent an emergency disable.

    Returns False and reports to stderr if history cannot be saved. A disk failure
    therefore means the record is incomplete, never silently reported complete.
    """
    global _WARNED
    try:
        default_logger().log(event,**fields)
        return True
    except Exception as exc:
        if not _WARNED:
            print(f'DIAGNOSTIC LOG FAILURE: history may be incomplete: {exc}',file=sys.stderr,flush=True)
            _WARNED=True
        return False


def close_default_logger():
    if _DEFAULT is not None: _DEFAULT.close()

atexit.register(close_default_logger)


def plot_static(times,series,output,*,ylabel='value',title='Recorded history',show=False):
    import matplotlib
    if not show: matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(11,6))
    for label,values in series.items(): ax.plot(times,values,label=label,linewidth=.9)
    ax.set(xlabel='Time (s)',ylabel=ylabel,title=title);ax.grid(alpha=.3);ax.legend()
    fig.tight_layout();fig.savefig(output,dpi=160)
    if show: plt.show()
    plt.close(fig)
    return str(output)


def _live_worker(samples,ready,closed,kind):
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        fig,axes=plt.subplots(2,1,sharex=True,figsize=(11,7))
        fig.canvas.manager.set_window_title('Motor diagnostics — common graph logger')
        rawline=axes[0].plot([],[],label='raw counts',linewidth=.8)[0]
        degree=axes[1].plot([],[],label='raw deviation',linewidth=.8)[0]
        filtered=axes[1].plot([],[],label=kind,linewidth=1.3)[0] if kind!='raw' else None
        axes[0].set_ylabel('Raw CAN counts');axes[0].ticklabel_format(useOffset=False,style='plain',axis='y')
        axes[1].set_ylabel('Deviation from startup (degrees)');axes[1].set_xlabel('Recorded elapsed time (s)')
        for ax in axes: ax.grid(alpha=.3);ax.legend()
        fig.text(.5,.01,'REST TEST: keep supported motor untouched. Full supplied history is logged separately.',ha='center',fontsize=9)
        title=fig.suptitle('Waiting for feedback')
        history=deque(maxlen=20000)
        def update(_):
            for _ in range(20000):
                try: history.append(samples.get_nowait())
                except Empty: break
            if not history: return
            latest=history[-1][0]
            while history and history[0][0]<latest-10: history.popleft()
            t=[r[0] for r in history];raw=[r[1] for r in history];deg=[r[2] for r in history]
            rawline.set_data(t,raw);degree.set_data(t,deg)
            other=[r[3] for r in history] if filtered else []
            if filtered: filtered.set_data(t,other)
            for ax,values,minimum in [(axes[0],raw,2),(axes[1],deg+other,2*DEGREES_PER_COUNT)]:
                lo,hi=min(values),max(values);margin=max(minimum,(hi-lo)*.1)
                ax.set_ylim(lo-margin,hi+margin);ax.set_xlim(max(0,latest-10),max(10,latest))
            # Last tuple entry is display drops, added by the shared sink.
            title.set_text(f'{kind} | raw {raw[-1]} | 10 s span {max(raw)-min(raw)} counts | display drops {history[-1][-1]}')
        animation=FuncAnimation(fig,update,interval=50,cache_frame_data=False)
        fig.canvas.mpl_connect('close_event',lambda _:closed.set())
        fig.tight_layout(rect=(0,.04,1,.94));ready.set();plt.show()
    finally: closed.set()


class _LiveSink:
    def __init__(self,queue,kind,logger=None): self.queue,self.kind,self.dropped,self.logger=queue,kind,0,logger
    def sample(self,elapsed_s,raw_count,raw_deg,filtered_deg=None):
        row=(elapsed_s,raw_count,raw_deg) if filtered_deg is None else (elapsed_s,raw_count,raw_deg,filtered_deg)
        self.put_nowait(row)
    def put_nowait(self,row):
        # Log BEFORE attempting display. Preserve original tuple and its metadata.
        if self.logger is None: record_event('live_sample',kind=self.kind,values=list(row))
        else: self.logger.log('live_sample',kind=self.kind,values=list(row))
        try: self.queue.put_nowait(tuple(row)+(self.dropped,))
        except Full: self.dropped+=1


@contextmanager
def plot_session(enabled,kind='raw',logger=None):
    """Common existing-reader adapter: yields (sample_sink, closed_event, process)."""
    if not enabled:
        yield None;return
    ctx=mp.get_context('spawn')
    queue,ready,closed=ctx.Queue(maxsize=20000),ctx.Event(),ctx.Event()
    process=ctx.Process(target=_live_worker,args=(queue,ready,closed,kind),daemon=True)
    process.start()
    try:
        deadline=time.monotonic()+15
        while not ready.wait(.05):
            if closed.is_set() or not process.is_alive() or time.monotonic()>deadline:
                raise RuntimeError('Common graph could not open; use Jetson graphical desktop')
        if logger is None: record_event('live_graph_open',kind=kind)
        else: logger.log('live_graph_open',kind=kind)
        yield _LiveSink(queue,kind,logger),closed,process
    finally:
        if process.is_alive(): process.terminate()
        process.join(timeout=2);queue.cancel_join_thread();queue.close()
        if logger is None: record_event('live_graph_closed',kind=kind)
        else: logger.log('live_graph_closed',kind=kind)


def save_comparison_plot(out, report,t,phase,values,target,evaluation,show):
    record_event('comparison_history', times=t, phase=phase, values=values,
                 target=[float(v) if math.isfinite(v) else None for v in target], evaluation=evaluation)
    import matplotlib
    if not show: matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(14,9))
    dynamic=phase=='dynamic'
    center=float(values['raw'][evaluation].mean())
    colors={'raw':'0.55','ema':'tab:blue','kalman':'tab:orange','lowpass':'tab:green'}
    for name,y in values.items():
        axes[0,0].plot(t[evaluation],y[evaluation]-center,label=name,color=colors[name],linewidth=.9)
        axes[0,1].plot(t[dynamic],y[dynamic],label=name,color=colors[name],linewidth=1)
        if name!='raw': axes[1,0].plot(t[dynamic],y[dynamic]-values['raw'][dynamic],label=name,color=colors[name])
    axes[0,1].plot(t[dynamic],target[dynamic],'k--',label='command (not ground truth)',linewidth=.8)
    axes[0,0].set(title='Static: held-out readings, common raw-mean reference',ylabel='Deviation (degrees)')
    axes[0,1].set(title='Dynamic: continuous replay',ylabel='Position from startup (degrees)')
    axes[1,0].set(title='Dynamic filter-minus-raw difference',ylabel='Difference (degrees)')
    for ax in [axes[0,0],axes[0,1],axes[1,0]]:
        ax.set_xlabel('Recorded elapsed time (s)');ax.grid(alpha=.3);ax.legend()
    names=list(report['metrics'])
    axes[1,1].bar(names,[report['metrics'][n]['static']['std_deg'] for n in names],color=[colors[n] for n in names])
    axes[1,1].set(title='Static standard deviation (lower is smoother)',ylabel='Degrees')
    fig.suptitle(report['selection']['result']+'\nConditional on stated lag/distortion budgets; not an accuracy claim',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.94))
    fig.savefig(out/'comparison.png',dpi=160)
    if show: plt.show()
    plt.close(fig)



def plot_history(history, *, show=False, single_turn=False):
    """Render a full JSONL run, including position commands and measured feedback."""
    import matplotlib
    if not show: matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    history=Path(history)
    if history.is_dir(): history=history/'events.jsonl'
    traces={}
    damaged=0
    def add(label,t,value):
        if value is None or not math.isfinite(value):return
        traces.setdefault(label,([],[]))
        value=value%360 if single_turn else value
        times,values=traces[label]
        if single_turn and values and abs(value-values[-1])>180:
            times.append(t);values.append(float('nan'))
        times.append(t);values.append(value)
    with history.open() as stream:
        for line in stream:
            try: event=json.loads(line)
            except json.JSONDecodeError:
                damaged+=1;continue
            data=event['fields']; t=event['elapsed_s']; kind=event['event']
            motor=data.get('motor_id','?'); channel=data.get('channel','')
            prefix=f'{channel} ID {motor}'
            if kind in ('feedback','recorded_feedback') and 'raw_count' in data:
                add(prefix+' raw',t,(data['raw_count']-32767.5)*DEGREES_PER_COUNT)
            elif kind=='beta_feedback':
                for key in ('ema_deg','kalman_deg','lowpass_deg'):
                    add(prefix+' '+key,t,data['sample'][key])
            elif kind=='filtered_feedback':
                add(prefix+' '+data.get('filter','filtered'),t,(data['filtered_count']-32767.5)*DEGREES_PER_COUNT)
            elif kind=='command_submitted' and 'payload_hex' in data:
                payload=bytes.fromhex(data['payload_hex'])
                if len(payload)==8 and payload[:7]!=b'\xff'*7:
                    kp=((payload[3]&15)<<8)|payload[4]
                    if kp: add(prefix+' commanded position',t,(((payload[0]<<8)|payload[1])-32767.5)*DEGREES_PER_COUNT)
    if not traces: raise ValueError('No supported position command/feedback events in this history')
    fig,ax=plt.subplots(figsize=(12,7))
    for name,(times,values) in traces.items(): ax.plot(times,values,label=name,linewidth=.8)
    ax.set(title=f'Full diagnostic history: {history.parent.name} (malformed rows: {damaged})',xlabel='Session elapsed time (s)',ylabel='Absolute output position (degrees)')
    ax.legend();ax.grid(alpha=.3);fig.tight_layout()
    output=history.parent/'history.png';fig.savefig(output,dpi=160)
    if show: plt.show()
    plt.close(fig)
    return str(output)


def browser_plot_javascript():
    """Shared browser graph renderer; all beta traces use this endpoint."""
    return r"""
window.drawMotorGraph = (canvas, points, keys) => {
 const dpr=window.devicePixelRatio||1, w=canvas.clientWidth, h=canvas.clientHeight;
 if(canvas.width!==w*dpr||canvas.height!==h*dpr){canvas.width=w*dpr;canvas.height=h*dpr;}
 const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
 const colors={raw_deg:'#78909c',lowpass_deg:'#dc8a25',ema_deg:'#008477',kalman_deg:'#7d61ac',target_deg:'#c64747'};
 const vals=points.flatMap(p=>keys.map(k=>p[k])).filter(Number.isFinite);
 c.font='12px system-ui';c.fillStyle='#657572';
 if(!vals.length){c.fillText('Encoder history appears after Enable / monitor.',24,h/2);return;}
 let lo=Math.min(...vals),hi=Math.max(...vals),pad=Math.max(.03,(hi-lo)*.12);lo-=pad;hi+=pad;
 const left=64,right=w-18,top=22,bottom=h-30;
 const t1=points[0].time,t2=Math.max(t1+.001,points[points.length-1].time);
 for(let i=0;i<5;i++){let y=top+(bottom-top)*i/4;c.strokeStyle='#e5ece8';c.beginPath();c.moveTo(left,y);c.lineTo(right,y);c.stroke();c.fillStyle='#657572';c.fillText((hi-(hi-lo)*i/4).toFixed(3),3,y+4);}
 for(const k of keys){c.strokeStyle=colors[k];c.lineWidth=k==='raw_deg'?1:1.6;c.beginPath();let fresh=true,previous=null;for(const p of points){if(!Number.isFinite(p[k])){fresh=true;previous=null;continue;}let x=left+(p.time-t1)/(t2-t1)*(right-left),y=bottom-(p[k]-lo)/(hi-lo)*(bottom-top);if(fresh||(previous!==null&&Math.abs(p[k]-previous)>180)){c.moveTo(x,y);fresh=false;}else c.lineTo(x,y);previous=p[k];}c.stroke();}
 c.fillStyle='#657572';c.fillText('Absolute motor-zero angle (deg)',left,13);c.fillText((t2-t1).toFixed(1)+' s shown · full history logged',left,h-8);
};
"""


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Examples: python3 -B graph_logger.py --help\npython3 -B graph_logger.py --clear-up  # deletes all logs AND encoder_records\nLibrary: with graph_logger("run") as log: log.log("feedback", raw_count=123)')
    parser.add_argument('-help',action='help',help='Show help and exit')
    parser.add_argument('--clear-up','-clear_up',action='store_true',help='Delete diagnostic_logs and encoder_records; code/config unchanged')
    parser.add_argument('--history',type=Path,help='Render a full events.jsonl file or session folder')
    parser.add_argument('--show',action='store_true',help='Open the history graph in the common Matplotlib GUI')
    args=parser.parse_args(argv)
    if args.clear_up and args.history: parser.error('Choose --clear-up or --history, not both')
    if args.clear_up: print(json.dumps(graph_logger(clear_up=True),indent=2))
    elif args.history: print(plot_history(args.history,show=args.show))
    else: parser.print_help()


if __name__=='__main__':
    try: main()
    except (OSError,RuntimeError) as exc: raise SystemExit(str(exc))
