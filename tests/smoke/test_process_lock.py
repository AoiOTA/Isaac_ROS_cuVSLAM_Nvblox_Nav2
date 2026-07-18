import sys
from pathlib import Path
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "isaac_sim"))

from jackal_sim.process_lock import (  # noqa: E402
    SimulatorAlreadyRunning,
    SimulatorProcessLock,
)


def test_simulator_lock_waits_then_reacquires_after_release(tmp_path: Path) -> None:
    path = tmp_path / "simulator.lock"
    owner = SimulatorProcessLock(path)
    owner.acquire()
    started = time.monotonic()
    with pytest.raises(SimulatorAlreadyRunning):
        SimulatorProcessLock(path, wait_seconds=0.05).acquire()
    assert time.monotonic() - started >= 0.05
    owner.release()
    follower = SimulatorProcessLock(path, wait_seconds=0.05)
    follower.acquire()
    follower.release()


def test_simulator_lock_rejects_negative_wait(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        SimulatorProcessLock(tmp_path / "simulator.lock", wait_seconds=-0.1)
