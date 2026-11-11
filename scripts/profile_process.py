# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Bound a coverage command while preserving separate, partial output files."""
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time


class CommandInterrupted(RuntimeError):
    """A controller signal must escape selectors' ordinary EINTR retry path."""
    def __init__(self, number):
        self.signal = number
        super().__init__(f"profile command interrupted by signal {number}")


def _retire(process, grace):
    # A signal can arrive just after wait() reaps a normally completed child.
    # Its PID is no longer pinned, so never signal that process group again.
    if process.returncode is not None:
        return
    # Do not reap before the final group signal: retaining our direct child's
    # PID prevents an unrelated process group from reusing this identity.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    time.sleep(grace)
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=5)


def run(command, *, cwd, env, directory, label, timeout=3600,
        output_limit=64 * 1024 * 1024, termination_grace=5):
    """Return ordinary captured output; timeout or overflow is always failure.

    Both pipes are drained concurrently into exclusive files before command
    completion. Bounds apply to these transcripts, not Cargo's object files.
    No tests, assertions, environment choices or coverage thresholds change.
    """
    if timeout <= 0 or output_limit <= 0 or termination_grace < 0:
        raise ValueError("invalid profile command bounds")
    if not label or Path(label).name != label:
        raise ValueError("profile command label must be a filename")
    paths = {name: Path(directory) / f"{label}.{name}" for name in ("stdout", "stderr")}
    process = None
    first_signal = None
    launching = False
    retiring = False
    reaping = False
    previous_handlers = {}

    def interrupted(number, _frame):
        nonlocal first_signal
        if first_signal is None:
            first_signal = number
            if not launching and not retiring and not reaping:
                raise CommandInterrupted(number)

    deadline = time.monotonic() + timeout
    with paths["stdout"].open("xb") as stdout, paths["stderr"].open("xb") as stderr:
        files = {"stdout": stdout, "stderr": stderr}
        sizes = {name: 0 for name in files}
        with selectors.DefaultSelector() as selector:
            try:
                for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                    previous_handlers[number] = signal.getsignal(number)
                    signal.signal(number, interrupted)
                # Defer raising during Popen construction until its child has
                # been assigned to our cleanup owner. No signal mask is passed
                # to Cargo or its children.
                launching = True
                process = subprocess.Popen(command, cwd=cwd, env=env,
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                           start_new_session=True)
                launching = False
                if first_signal is not None:
                    raise CommandInterrupted(first_signal)
                for name in files:
                    stream = getattr(process, name)
                    os.set_blocking(stream.fileno(), False)
                    selector.register(stream, selectors.EVENT_READ, name)
                while selector.get_map():
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise subprocess.TimeoutExpired(command, timeout)
                    for key, _ in selector.select(min(remaining, 0.1)):
                        chunk = os.read(key.fd, 65536)
                        if not chunk:
                            selector.unregister(key.fileobj)
                            continue
                        name = key.data
                        available = output_limit - sizes[name]
                        files[name].write(chunk[:available])
                        files[name].flush()
                        sizes[name] += min(len(chunk), available)
                        if len(chunk) > available:
                            raise RuntimeError(f"profile {name} exceeded {output_limit} bytes; partial files: {paths}")
                # Observe without reaping so an interruption cannot invalidate
                # the process-group PID pin between waitpid and returncode.
                while os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT) is None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise subprocess.TimeoutExpired(command, timeout)
                    time.sleep(min(remaining, 0.01))
                reaping = True
                result = process.wait(timeout=5)
                reaping = False
                if first_signal is not None:
                    raise CommandInterrupted(first_signal)
            except BaseException as error:
                retiring = True
                if process is not None:
                    _retire(process, termination_grace)
                if first_signal is not None:
                    error.add_note(f"first profile interruption signal: {first_signal}")
                raise
            finally:
                for number, handler in previous_handlers.items():
                    signal.signal(number, handler)
                if process is not None:
                    for name in files:
                        stream = getattr(process, name)
                        if stream is not None:
                            stream.close()
    # Preserve the previous subprocess.run(text=True) channel semantics and the
    # existing caller's stdout+stderr test-output.log representation.
    return subprocess.CompletedProcess(command, result,
                                       paths["stdout"].read_text(), paths["stderr"].read_text())
