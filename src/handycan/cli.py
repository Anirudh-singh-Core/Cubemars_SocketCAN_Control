"""Explicit CAN status, communication tests, and raw monitoring. No auto setup."""
import argparse
import json
from pathlib import Path
import sys
import time
from .config import Motor
from .session import Session
from .diagnostics import can_test
from .interface import inspect_interface

def load_config(path):
    data=json.loads(Path(path).read_text());channel=data.get('channel','can0')
    motors=[Motor.from_degrees(channel=channel,**row) for row in data['motors']]
    return motors,channel,data.get('control_hz',1000)

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('-help',action='help')
    sub=p.add_subparsers(dest='action',required=True)
    status=sub.add_parser('status',help='Read interface status; no motor commands')
    status.add_argument('-help',action='help');status.add_argument('--channel',default='can0')
    for name in ('test','monitor'):
        q=sub.add_parser(name,help='Bounded CAN test' if name=='test' else 'Enable neutral polling and print encoder')
        q.add_argument('-help',action='help');q.add_argument('--config',required=True)
        q.add_argument('--id',type=int,default=1)
        q.add_argument('--active',action='store_true',help='Explicitly enable motor at zero gains; disable on exit')
        if name=='test':q.add_argument('--report',type=Path,default=Path('logs/can_test_latest.json'))
        else:q.add_argument('--seconds',type=float,default=10)
    args=p.parse_args(argv)
    try:
        if args.action=='status':
            info=inspect_interface(args.channel);print(json.dumps(info,indent=2));return 0 if info['ready'] else 1
        motors,channel,hz=load_config(args.config)
        if args.id not in [m.limits.motor_id for m in motors]:raise ValueError('ID is not in configuration')
        if args.action=='monitor':
            if not args.active:raise ValueError('Monitoring requires --active; it enables the motor')
            if not 0<args.seconds<=3600:raise ValueError('seconds must be 0..3600')
        with Session(motors,channel=channel,control_hz=hz) as session:
            if args.action=='test':
                report=can_test(session,args.id,active=args.active)
                args.report.parent.mkdir(parents=True,exist_ok=True)
                args.report.write_text(json.dumps(report,indent=2)+'\n')
                print('Pass' if report['result']=='PASS' else 'Fail')
                return 0 if report['result']=='PASS' else 1
            session.enable(args.id);end=time.monotonic()+args.seconds
            while time.monotonic()<end:
                state=session.snapshot()
                if state['fault']:raise RuntimeError(state['fault'])
                if state['axes'][args.id]['state']=='MONITORING':
                    print(json.dumps(session.get_encoder_feedback(args.id)),flush=True)
                time.sleep(.05)
        return 0
    except KeyboardInterrupt:return 130
    except Exception as exc:
        if args.action=='test':print('Fail')
        print(f'{type(exc).__name__}: {exc}',file=sys.stderr);return 1

if __name__=='__main__':raise SystemExit(main())
