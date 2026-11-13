#!/usr/bin/env python3
"""Beta V2 browser GUI — without Continuous API controls.

Launch with .venv/bin/python -B Motor_Controller_beta_V2.py
Starts disconnected, sends no motor commands. No simulation mode or boot setup.
Local browser: http://127.0.0.1:8767. For remote access, forward port 8767 over SSH.
Closing a browser tab does not stop an active motor; use STOP ALL before leaving.
"""
import argparse
import signal

def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-help',action='help');p.add_argument('--port',type=int,default=8767)
    args=p.parse_args()
    try:
        from motor_controller_beta_v2.app import Application
        from motor_controller_beta_v2.web import make_server
    except ModuleNotFoundError as exc:
        raise SystemExit(f'{exc}. Use the installed .venv/bin/python interpreter.') from exc
    app=Application();server=make_server(app,port=args.port)
    print(f'Beta V2: http://127.0.0.1:{server.server_port} — REAL HARDWARE, DISCONNECTED; no motor commands sent.',flush=True)
    try:server.serve_forever(poll_interval=.2)
    finally:server.server_close();app.close()

if __name__=='__main__':
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    try:main()
    except KeyboardInterrupt:pass
