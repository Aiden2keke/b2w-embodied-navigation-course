"""Loopback-only HTTP adapter. Fixed routes, no filesystem writes except new exports."""
import io
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
import secrets
from urllib.parse import urlsplit

from .core import DISCLAIMER

ASSETS=Path(__file__).resolve().parent


def create_server(editor,output_root,port=8765):
    token=secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def send(self,status,payload,mime='application/json'):
            if mime=='application/json': payload=json.dumps(payload,allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type',mime)
            self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; connect-src 'self'; img-src 'self'; style-src 'self'; script-src 'self'; frame-ancestors 'none'")
            self.end_headers(); self.wfile.write(payload)

        def local_host(self):
            return self.headers.get('Host')==f'127.0.0.1:{self.server.server_port}'

        def do_GET(self):
            if not self.local_host(): return self.send(403,dict(error='Use the printed 127.0.0.1 URL'))
            path=urlsplit(self.path).path
            if path=='/bootstrap':
                return self.send(200,dict(token=token,initial=editor.initial,map=editor.map.meta,
                    occupancy_policy=editor.map.occupancy_policy,identity=editor.map.identity,sources=editor.sources,footprint=editor.footprint,
                    costmaps={name:cm.meta for name,cm in editor.costmaps.items()}))
            if path=='/map.png' or path in [f'/{name}-costmap.png' for name in editor.costmaps]:
                im=editor.map.image if path=='/map.png' else editor.costmaps[path.split('-')[0][1:]].image
                buf=io.BytesIO();im.save(buf,format='PNG');return self.send(200,buf.getvalue(),'image/png')
            assets={'/':('index.html','text/html; charset=utf-8'),'/app.mjs':('app.mjs','text/javascript'),
                    '/model.mjs':('model.mjs','text/javascript'),'/style.css':('style.css','text/css')}
            if path not in assets:return self.send(404,dict(error='Unknown route'))
            name,mime=assets[path];return self.send(200,(ASSETS/name).read_bytes(),mime)

        def do_POST(self):
            origin=self.headers.get('Origin')
            if (not self.local_host() or self.headers.get('X-Editor-Token')!=token
                    or origin not in (None,f'http://127.0.0.1:{self.server.server_port}')):
                return self.send(403,dict(error='Invalid editor token or origin'))
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=1_000_000: return self.send(413,dict(error='Request size out of bounds'))
                if self.headers.get('Content-Type')!='application/json': return self.send(415,dict(error='Require JSON'))
                request=json.loads(self.rfile.read(size)); state=request['state']
                editor.assert_sources();editor.validate(state)
                if self.path=='/validate': return self.send(200,dict(valid=True))
                if self.path=='/check':
                    result=dict(poses=editor.check_state(state),warning=DISCLAIMER)
                    if request.get('sweep'):
                        poses={g['name']:[*g['position'],g['yaw_rad']] for g in state['goals']};poses.update(state['markers'])
                        result['sweep']=editor.sweep(poses[request['selected']],state['polygons'])
                    return self.send(200,result)
                if self.path=='/export':return self.send(200,dict(path=str(editor.export(state,output_root)),warning=DISCLAIMER))
                return self.send(404,dict(error='Unknown route'))
            except (ValueError,KeyError,TypeError,OSError,OverflowError) as error:
                return self.send(400,dict(error=str(error)))

        def log_message(self,fmt,*args):
            pass
    return HTTPServer(('127.0.0.1',port),Handler)
