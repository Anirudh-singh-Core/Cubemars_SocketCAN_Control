"""Loopback-only beta V2 HTTP server. GET requests never activate motor hardware."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
import json
import secrets
from graph_logger import browser_plot_javascript

STATIC=Path(__file__).resolve().parent/'static'

def make_server(app,host='127.0.0.1',port=8767):
    if host not in ('localhost','127.0.0.1'):raise ValueError('Use loopback and an SSH tunnel for remote access')
    token=secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def setup(self):
            super().setup();self.connection.settimeout(3)
        def reply(self,status,data,content_type='application/json'):
            body=data if isinstance(data,bytes) else json.dumps(data,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff');self.send_header('X-Frame-Options','DENY')
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def trusted(self):return self.headers.get('Host') in (f'localhost:{self.server.server_port}',f'127.0.0.1:{self.server.server_port}')
        def do_GET(self):
            if not self.trusted():return self.reply(403,{'error':'Invalid Host'})
            path=urlsplit(self.path).path
            if path=='/':return self.reply(200,(STATIC/'index.html').read_text().replace('__TOKEN__',token).encode(),'text/html; charset=utf-8')
            if path=='/graph.js':
                js=browser_plot_javascript().replace('full history logged','history queued; check log-drop counter')
                return self.reply(200,js.encode(),'text/javascript; charset=utf-8')
            if path=='/api/status':return self.reply(200,app.snapshot())
            if path=='/api/report':return self.reply(200,(app.snapshot().get('job') or {}).get('report'))
            return self.reply(404,{'error':'Not found'})
        def do_POST(self):
            if not self.trusted() or self.headers.get('X-Control-Token')!=token:return self.reply(403,{'error':'Invalid control token/host'})
            origin=self.headers.get('Origin')
            if origin and origin!='http://'+self.headers.get('Host',''):return self.reply(403,{'error':'Cross-origin commands refused'})
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':return self.reply(415,{'error':'JSON required'})
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=65536:raise ValueError('Invalid request size')
                data=json.loads(self.rfile.read(length),parse_constant=lambda _:(_ for _ in ()).throw(ValueError('Non-finite number')))
                if not isinstance(data,dict):raise ValueError('JSON object required')
                path=urlsplit(self.path).path
                if path=='/api/stop':app.stop();return self.reply(202,{'stop_requested':True})
                if path!='/api/action':return self.reply(404,{'error':'Not found'})
                action=data.pop('action');result=app.action(action,data)
                return self.reply(200,{'result':result})
            except Exception as exc:return self.reply(400,{'error':str(exc) or type(exc).__name__})
    return ThreadingHTTPServer((host,port),Handler)
