"""Private bounded grid IPC. No ROS entities or command publication in this process."""
import pickle
import socket
import struct
import time

MAX_MESSAGE=128*1024*1024


def remaining(sock,deadline):
    if deadline is not None:
        timeout=deadline-time.monotonic()
        if timeout<=0:raise TimeoutError('grid_worker_deadline')
        sock.settimeout(timeout)


def send(sock,value,*,deadline=None):
    data=pickle.dumps(value,protocol=5)
    if len(data)>MAX_MESSAGE:raise ValueError('grid_ipc_oversized')
    remaining(sock,deadline)
    sock.sendall(struct.pack('!I',len(data))+data)


def receive(sock,*,deadline=None):
    def exact(size):
        data=bytearray(size);view=memoryview(data);offset=0
        while offset<size:
            remaining(sock,deadline)
            count=sock.recv_into(view[offset:])
            if not count:raise EOFError('grid_worker_disconnected')
            offset+=count
        return data
    size=struct.unpack('!I',exact(4))[0]
    if size>MAX_MESSAGE:raise ValueError('grid_ipc_oversized')
    value=pickle.loads(exact(size))
    remaining(sock,deadline)
    return value


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
            entry=guard.prepare(name,grid,**metadata)
            send(sock,('result',name,generation,metadata,entry,guard.failure))
    except (EOFError,BrokenPipeError,ConnectionResetError):pass
    finally:sock.close()


if __name__=='__main__':
    import sys
    main(int(sys.argv[1]))
