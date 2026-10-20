"""Verify an explicitly permitted external deployment, never assume ROOT/.deps."""
import json
import os
from pathlib import Path
from .contract import digest


def load_deployment(path):
    path=Path(path)
    if path.stat().st_size>65536:raise ValueError('Oversized session deployment')
    value=json.loads(path.read_text())
    if value.get('kind')!='stage4_session_deployment' or value.get('version')!=1 or not value.get('session_id'):
        raise ValueError('Explicit versioned session deployment required')
    if not Path(value['guard_manifest']).is_absolute():raise ValueError('Absolute guard manifest required')
    return value


def verify_controller(deployment,policy,*,proc_root=Path('/proc')):
    try:
        c=deployment['controller'];expected=c['process'];p=proc_root/str(expected['pid'])
        prefix=Path(c['prefix']).resolve();exe=Path(c['executable']).resolve();lib=Path(c['library']).resolve()
        if (exe!=prefix/'lib/nav2_controller/controller_server' or lib!=prefix/'lib/libcontroller_server_core.so' or
                c['exe_sha256']!=policy['exe_sha256'] or c['library_sha256']!=policy['library_sha256'] or
                digest(exe)!=policy['exe_sha256'] or digest(lib)!=policy['library_sha256']):raise ValueError('Unpinned controller deployment')
        def identity():
            fields=(p/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]=='Z':raise ValueError('Controller dead')
            return dict(pid=int(p.name),start_ticks=fields[19],pgid=int(fields[2]),uid=p.stat().st_uid)
        if identity()!=expected or (p/'exe').resolve()!=exe:raise ValueError('Controller process identity changed')
        if not c['owner_token'] or ('B2W_NARROW_OWNER='+c['owner_token']).encode() not in (p/'environ').read_bytes().split(b'\0'):
            raise ValueError('Controller ownership mismatch')
        mapped=[];st=lib.stat()
        for line in (p/'maps').read_text().splitlines():
            fields=line.split(maxsplit=5)
            if len(fields)<6 or 'libcontroller_server_core.so' not in fields[5]:continue
            if fields[5]!=str(lib) or int(fields[4])!=st.st_ino:
                raise ValueError('Foreign/deleted/replaced controller library mapping')
            major,minor=(int(v,16) for v in fields[3].split(':'))
            if (major,minor)!=(os.major(st.st_dev),os.minor(st.st_dev)):raise ValueError('Controller mapped device mismatch')
            mapped.append(fields[5])
        if not mapped or identity()!=expected:raise ValueError('Missing mapping or reused controller PID')
        return dict(pid=expected['pid'],start_ticks=expected['start_ticks'],pgid=expected['pgid'],
                    executable=str(exe),library=str(lib),exe_sha256=policy['exe_sha256'],library_sha256=policy['library_sha256'])
    except (OSError,KeyError,TypeError,IndexError) as exc:
        raise ValueError('Incomplete/unavailable explicit controller deployment: '+str(exc)) from exc
