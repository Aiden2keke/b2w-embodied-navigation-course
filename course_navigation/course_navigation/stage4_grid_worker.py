"""Private bounded grid IPC. No ROS entities or command publication in this process."""
import pickle
import socket
import struct
import time
import array
import mmap
import os

MAX_MESSAGE=128*1024*1024
_FRAME=struct.Struct('!IIII')
_BUFFER_SIZE=struct.Struct('!Q')
_MAX_BUFFERS=64
_FLAG_SHARED=1


def _serialize(value):
    """Pickle protocol 5 out-of-band buffers to avoid a second large byte copy."""
    buffers=[]
    payload=pickle.dumps(value,protocol=5,buffer_callback=buffers.append)
    return payload,[item.raw() for item in buffers]


def remaining(sock,deadline):
    if deadline is not None:
        timeout=deadline-time.monotonic()
        if timeout<=0:raise TimeoutError('grid_worker_deadline')
        sock.settimeout(timeout)


def send(sock,value,*,deadline=None,shared_buffers=False):
    data,buffers=_serialize(value)
    if len(buffers)>_MAX_BUFFERS:raise ValueError('grid_ipc_oversized')
    total=len(data)+sum(buffer.nbytes for buffer in buffers)
    if total>MAX_MESSAGE:raise ValueError('grid_ipc_oversized')
    use_shared=bool(shared_buffers and buffers and sum(buffer.nbytes for buffer in buffers)>0)
    shared_fd=None;shared_map=None;shared_stage_ns=0
    try:
        if use_shared:
            shared_started=time.monotonic()
            shared_fd=os.memfd_create('b2w-stage4-grid',os.MFD_CLOEXEC)
            os.ftruncate(shared_fd,total-len(data))
            shared_map=mmap.mmap(shared_fd,total-len(data),access=mmap.ACCESS_WRITE)
            offset=0
            for buffer in buffers:
                shared_map[offset:offset+buffer.nbytes]=buffer
                offset+=buffer.nbytes
            shared_stage_ns=int((time.monotonic()-shared_started)*1e9)
        header=_FRAME.pack(len(data),len(buffers),_FLAG_SHARED if use_shared else 0,shared_stage_ns)
        remaining(sock,deadline)
        if use_shared:
            rights=array.array('i',[shared_fd])
            sent=sock.sendmsg([header],[(socket.SOL_SOCKET,socket.SCM_RIGHTS,rights)])
            if sent<len(header):
                remaining(sock,deadline);sock.sendall(header[sent:])
        else:sock.sendall(header)
        remaining(sock,deadline);sock.sendall(data)
        for buffer in buffers:
            remaining(sock,deadline);sock.sendall(_BUFFER_SIZE.pack(buffer.nbytes))
            if not use_shared:
                remaining(sock,deadline);sock.sendall(buffer)
    finally:
        if shared_map is not None:shared_map.close()
        if shared_fd is not None:os.close(shared_fd)


def receive(sock,*,deadline=None,phase_times=None):
    def exact(size):
        data=bytearray(size);view=memoryview(data);offset=0
        while offset<size:
            remaining(sock,deadline)
            count=sock.recv_into(view[offset:])
            if not count:raise EOFError('grid_worker_disconnected')
            offset+=count
        return data
    read_started=time.monotonic();received_fd=None;shared_map=None
    try:
        ancillary=[]
        if hasattr(sock,'recvmsg'):
            remaining(sock,deadline)
            header_bytes,ancillary,flags,_=sock.recvmsg(_FRAME.size,socket.CMSG_SPACE(array.array('i').itemsize))
            descriptors=[]
            for level,kind,raw in ancillary:
                if level==socket.SOL_SOCKET and kind==socket.SCM_RIGHTS:
                    received=array.array('i');received.frombytes(raw[:len(raw)-len(raw)%received.itemsize]);descriptors.extend(received)
            if len(descriptors)>1:
                for fd in descriptors:os.close(fd)
                descriptors=[]
                raise ValueError('grid_ipc_unexpected_descriptors')
            received_fd=descriptors[0] if descriptors else None
            if flags & getattr(socket,'MSG_CTRUNC',0):raise ValueError('grid_ipc_ancillary_truncated')
            if len(header_bytes)<_FRAME.size:header_bytes+=exact(_FRAME.size-len(header_bytes))
        else:header_bytes=exact(_FRAME.size)
        size,count,frame_flags,shared_stage_ns=_FRAME.unpack(header_bytes)
        if size>MAX_MESSAGE or count>_MAX_BUFFERS or frame_flags & ~_FLAG_SHARED:
            raise ValueError('grid_ipc_oversized')
        use_shared=bool(frame_flags&_FLAG_SHARED)
        if phase_times is not None and use_shared:
            phase_times['shared_buffer_stage_s']=shared_stage_ns/1e9
        if use_shared!=(received_fd is not None):raise ValueError('grid_ipc_descriptor_mismatch')
        payload=exact(size)
        if use_shared:
            sizes=[];total=size
            for _ in range(count):
                buffer_size=_BUFFER_SIZE.unpack(exact(_BUFFER_SIZE.size))[0]
                total+=buffer_size
                if total>MAX_MESSAGE:raise ValueError('grid_ipc_oversized')
                sizes.append(buffer_size)
            file_size=total-size
            if os.fstat(received_fd).st_size!=file_size:raise ValueError('grid_ipc_shared_size_mismatch')
            shared_map=mmap.mmap(received_fd,file_size,access=mmap.ACCESS_READ)
            os.close(received_fd);received_fd=None
            buffers=[];offset=0
            for buffer_size in sizes:
                buffers.append(memoryview(shared_map)[offset:offset+buffer_size]);offset+=buffer_size
        else:
            buffers=[];total=size
            for _ in range(count):
                buffer_size=_BUFFER_SIZE.unpack(exact(_BUFFER_SIZE.size))[0]
                total+=buffer_size
                if total>MAX_MESSAGE:raise ValueError('grid_ipc_oversized')
                buffers.append(exact(buffer_size))
        if phase_times is not None:phase_times['read_s']=time.monotonic()-read_started
        decode_started=time.monotonic()
        value=pickle.loads(payload,buffers=buffers)
        remaining(sock,deadline)
        if phase_times is not None:phase_times['decode_s']=time.monotonic()-decode_started
        return value
    finally:
        if phase_times is not None:phase_times.setdefault('read_s',time.monotonic()-read_started)
        if received_fd is not None:os.close(received_fd)
        # On successful decode, NumPy's memoryview bases retain the mmap. On
        # decode failure this local is the last reference and releases it.
        if shared_map is not None:del shared_map


def main(fd):
    # A private inherited socket to the owned parent, never a listening endpoint.
    from .stage4_guard import FinalCommandGuard
    sock=socket.socket(fileno=fd)
    try:
        guard=FinalCommandGuard(receive(sock))
        send(sock,('ready',))
        while True:
            job=receive(sock)
            if job is None:return
            name,generation,grid,metadata=job
            started=time.monotonic()
            entry=guard.prepare(name,grid,**metadata)
            send(sock,('prepared',name,generation,time.monotonic()-started))
            send(sock,('result',name,generation,metadata,entry,guard.failure),shared_buffers=True)
    except (EOFError,BrokenPipeError,ConnectionResetError):pass
    finally:sock.close()


if __name__=='__main__':
    import sys
    main(int(sys.argv[1]))
