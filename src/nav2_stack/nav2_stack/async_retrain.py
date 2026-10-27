"""Runs dynamics_retrain.retrain() in a background process, so autonomous_trials
can keep planning and driving while the dynamics model trains (--async-training).

A separate process rather than a thread: training holds the GIL in its Python
loop, a crash or CUDA error can't take the trial loop down with it, and its GPU
memory is released as soon as it exits. The "spawn" start method is required --
a forked child would inherit the parent's rclpy state and (if any) CUDA
context, neither of which is fork-safe.

The process is not a daemon: DataLoader(num_workers=2) starts children of its
own, which daemonic processes are not allowed to do. The caller must kill() it
on abort instead.
"""

import multiprocessing as mp
import os
import sys
import time
import traceback
from pathlib import Path


def _worker(kwargs: dict, log_path: str, conn):
    # A spawned child inherits "spawn" as its default start method, so the
    # DataLoader(num_workers=2) inside retrain() would spawn fresh interpreters
    # (re-importing torch) for its workers every epoch -- measured 5-10x slower
    # training. This process is fresh (no rclpy, no CUDA yet), so forking its
    # loader workers is as safe as in the serial path.
    mp.set_start_method("fork", force=True)
    # Everything the training prints (including DataLoader workers and CUDA
    # warnings) goes to its own log file instead of the trial terminal
    log = open(log_path, "w", buffering=1)
    os.dup2(log.fileno(), 1)
    os.dup2(log.fileno(), 2)
    sys.stdout = sys.stderr = log
    try:
        from nav2_stack import dynamics_retrain
        result = dynamics_retrain.retrain(**kwargs)
        conn.send(("ok", result))
    except BaseException as e:  # noqa: BLE001 -- report anything, including KeyboardInterrupt
        traceback.print_exc()
        conn.send(("error", f"{type(e).__name__}: {e}"))
    finally:
        conn.close()
        log.flush()


class BackgroundRetrain:
    """One dynamics_retrain.retrain(**kwargs) call running in a spawned process."""

    def __init__(self, kwargs: dict, log_path: Path):
        ctx = mp.get_context("spawn")
        self._recv, send = ctx.Pipe(duplex=False)
        self.proc = ctx.Process(target=_worker, args=(kwargs, str(log_path), send),
                                name="dynamics_retrain", daemon=False)
        self.t_start = time.monotonic()
        self.t_end = None
        self.log_path = log_path
        self.proc.start()
        send.close()  # only the child writes; lets recv() see EOF if the child dies

    def done(self) -> bool:
        return self._recv.poll() or not self.proc.is_alive()

    def elapsed(self) -> float:
        return (self.t_end or time.monotonic()) - self.t_start

    def wait(self) -> dict:
        """Blocks until training finishes. Returns retrain()'s result dict, or raises
        RuntimeError if it failed or the process died without reporting."""
        try:
            status, payload = self._recv.recv()
        except EOFError:
            status, payload = "error", None
        self.proc.join(timeout=30)
        self.t_end = time.monotonic()
        if status == "ok":
            return payload
        if payload is None:
            payload = f"training process exited with code {self.proc.exitcode} without a result"
        raise RuntimeError(f"{payload} (see {self.log_path})")

    def kill(self):
        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(timeout=10)
            if self.proc.is_alive():
                self.proc.kill()
                self.proc.join(timeout=5)
