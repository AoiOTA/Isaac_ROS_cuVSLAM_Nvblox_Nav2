"""A project-scoped, non-destructive process lock for the simulator."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import socket
import time


class SimulatorAlreadyRunning(RuntimeError):
    """Raised when this repository already owns the simulator lock."""


class SimulatorProcessLock:
    """Hold an advisory lock without inspecting or terminating other processes."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._stream = None

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            stream.seek(0)
            owner = stream.read().strip() or "owner information unavailable"
            stream.close()
            raise SimulatorAlreadyRunning(
                f"this project already has a running simulator: {owner}"
            ) from exc

        stream.seek(0)
        stream.truncate()
        stream.write(
            f"pid={os.getpid()} host={socket.gethostname()} started_unix={time.time():.6f}\n"
        )
        stream.flush()
        os.fsync(stream.fileno())
        self._stream = stream

    def release(self) -> None:
        if self._stream is None:
            return
        fcntl.flock(self._stream.fileno(), fcntl.LOCK_UN)
        self._stream.close()
        self._stream = None

    def __enter__(self) -> "SimulatorProcessLock":
        self.acquire()
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        self.release()
