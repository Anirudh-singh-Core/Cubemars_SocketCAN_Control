"""Bounded asynchronous JSONL history; the CAN worker never performs disk I/O."""
import json
import queue
import threading
import time
from pathlib import Path

class Journal:
    def __init__(self,path,capacity=20000):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.queue=queue.Queue(capacity);self.dropped=0;self.error=None
        self.stop=threading.Event()
        # Fail before any CAN operation if the output file cannot be opened.
        self.file=self.path.open('a',buffering=65536)
        self.thread=threading.Thread(target=self._run,name='can-journal',daemon=True)
        self.thread.start()
    def record(self,event,**data):
        row=dict(event=event,monotonic=time.monotonic(),wall_time=time.time(),**data)
        try:self.queue.put_nowait(row)
        except queue.Full:self.dropped+=1
    def _run(self):
        last=time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:row=self.queue.get(timeout=.1)
                except queue.Empty:row=None
                if row is not None:self.file.write(json.dumps(row,allow_nan=False)+'\n')
                if time.monotonic()-last>=1:self.file.flush();last=time.monotonic()
        except Exception as exc:self.error=str(exc)
        finally:self.file.close()
    def close(self):
        self.stop.set();self.thread.join(timeout=3)
        if self.thread.is_alive():self.error='Logger shutdown timed out'
