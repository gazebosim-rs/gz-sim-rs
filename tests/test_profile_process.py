# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Actual bounded child-process controls; no Cargo, SDK or product execution."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("profile_process", ROOT / "scripts/profile_process.py")
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)


class ProfileProcessTests(unittest.TestCase):
    def invoke(self, source, directory, **bounds):
        return profile.run([sys.executable, "-c", source], cwd=directory,
                           env=dict(os.environ, PROFILE_CONTROL="preserved"),
                           directory=directory, label="control", **bounds)

    def test_success_and_failure_keep_channels_exit_environment_and_cwd(self):
        for code in (0, 7):
            with self.subTest(code=code), tempfile.TemporaryDirectory() as directory:
                source = ("import os,sys; from pathlib import Path; "
                          "print(os.environ['PROFILE_CONTROL']); print(Path.cwd()); "
                          f"print('diagnostic',file=sys.stderr); sys.exit({code})")
                result = self.invoke(source, directory, timeout=10)
                self.assertEqual(result.returncode, code)
                self.assertEqual(result.stdout, f"preserved\n{directory}\n")
                self.assertEqual(result.stderr, "diagnostic\n")
                for name in ("stdout", "stderr"):
                    self.assertEqual((Path(directory) / f"control.{name}").read_text(),
                                     getattr(result, name))

    def test_timeout_retains_partial_output_and_reaps_a_child_ignoring_term(self):
        with tempfile.TemporaryDirectory() as directory:
            source = ("import os,signal,time; from pathlib import Path; "
                      "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                      "Path('pid').write_text(str(os.getpid())); "
                      "print('ready',flush=True); time.sleep(60)")
            with self.assertRaises(subprocess.TimeoutExpired):
                self.invoke(source, directory, timeout=2, termination_grace=0.1)
            pid = int((Path(directory) / "pid").read_text())
            self.assertEqual((Path(directory) / "control.stdout").read_text(), "ready\n")
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)

    def test_output_overflow_is_failure_and_retained_channel_has_a_hard_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "exceeded"):
                self.invoke("import os; os.write(1,b'x'*8192)", directory,
                            timeout=10, output_limit=1024, termination_grace=0)
            self.assertEqual((Path(directory) / "control.stdout").read_bytes(), b"x" * 1024)

    def test_both_pipes_are_drained_without_waiting_for_the_other_channel(self):
        with tempfile.TemporaryDirectory() as directory:
            source = "import os; os.write(2,b'e'*1048576); os.write(1,b'o'*1048576)"
            result = self.invoke(source, directory, timeout=10)
            self.assertEqual(result.returncode, 0)
            self.assertEqual(result.stdout, "o" * 1048576)
            self.assertEqual(result.stderr, "e" * 1048576)

    def test_closed_pipes_do_not_turn_a_live_child_into_success(self):
        with tempfile.TemporaryDirectory() as directory:
            source = ("import os,time; from pathlib import Path; "
                      "Path('pid').write_text(str(os.getpid())); "
                      "os.close(1); os.close(2); time.sleep(60)")
            with self.assertRaises(subprocess.TimeoutExpired):
                self.invoke(source, directory, timeout=2, termination_grace=0.1)
            pid = int((Path(directory) / "pid").read_text())
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)

    def test_existing_transcript_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "control.stdout"
            path.write_text("original")
            with self.assertRaises(FileExistsError):
                self.invoke("raise SystemExit('must not execute')", directory)
            self.assertEqual(path.read_text(), "original")

    def test_controller_term_and_repeated_hup_retire_the_child_before_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            helper = str(ROOT / "scripts/profile_process.py")
            child = ("import os,signal,time; from pathlib import Path; "
                     "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
                     "Path('pid').write_text(str(os.getpid())); print('ready',flush=True); time.sleep(60)")
            controller = f"""import importlib.util,os,sys
from pathlib import Path
spec=importlib.util.spec_from_file_location('profile_process',{helper!r})
profile=importlib.util.module_from_spec(spec); spec.loader.exec_module(profile)
retire=profile._retire
def observed_retire(process,grace):
    Path('retiring').write_text(str(process.pid))
    retire(process,grace)
profile._retire=observed_retire
try:
    profile.run([sys.executable,'-c',{child!r}],cwd=Path.cwd(),env=os.environ.copy(),directory=Path.cwd(),label='control',timeout=60,termination_grace=1)
except profile.CommandInterrupted as error:
    print(str(error),flush=True)
    raise SystemExit(23)
raise SystemExit(99)
"""
            command = subprocess.Popen([sys.executable, "-c", controller], cwd=root,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            pid, identity = None, None
            def until_file(name):
                deadline = time.monotonic() + 5
                while not (root / name).exists():
                    self.assertIsNone(command.poll(), "controller exited before checkpoint")
                    self.assertLess(time.monotonic(), deadline, name)
                    time.sleep(0.01)
            try:
                until_file("pid")
                pid = int((root / "pid").read_text())
                identity = Path(f"/proc/{pid}/stat").read_text().split(") ", 1)[1].split()[19]
                command.send_signal(signal.SIGTERM)
                until_file("retiring")
                command.send_signal(signal.SIGHUP)
                stdout, stderr = command.communicate(timeout=10)
                self.assertEqual(command.returncode, 23, stderr.decode())
                self.assertIn(f"interrupted by signal {signal.SIGTERM}", stdout.decode())
                with self.assertRaises(ProcessLookupError):
                    os.kill(pid, 0)
                self.assertEqual((root / "control.stdout").read_text(), "ready\n")
            finally:
                if command.poll() is None:
                    command.kill()
                command.communicate(timeout=5)
                status = Path(f"/proc/{pid}/stat") if pid else None
                if status is not None and status.exists():
                    actual = status.read_text().split(") ", 1)[1].split()[19]
                    if actual == identity:
                        os.killpg(pid, signal.SIGKILL)

    def test_signal_after_normal_reap_never_signals_an_unpinned_group(self):
        with tempfile.TemporaryDirectory() as directory:
            spawn = profile.subprocess.Popen
            def finish_then_signal(*args, **kwargs):
                process = spawn(*args, **kwargs)
                wait = process.wait
                def interrupted_wait(*wait_args, **wait_kwargs):
                    result = wait(*wait_args, **wait_kwargs)
                    os.kill(os.getpid(), signal.SIGTERM)
                    return result
                process.wait = interrupted_wait
                return process
            previous = {number: signal.getsignal(number)
                        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
            with patch.object(profile.subprocess, "Popen", side_effect=finish_then_signal), \
                 patch.object(profile.os, "killpg") as group_signal:
                with self.assertRaises(profile.CommandInterrupted):
                    self.invoke("print('complete')", directory, timeout=10)
                group_signal.assert_not_called()
            self.assertEqual({number: signal.getsignal(number) for number in previous}, previous)


if __name__ == "__main__":
    unittest.main()
