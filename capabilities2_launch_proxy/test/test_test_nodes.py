import pathlib
import signal
import subprocess
import sys
import time


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
TEST_NODES_DIR = PACKAGE_ROOT / 'test_nodes'


def _run_and_interrupt(script_name: str, *args: str):
    process = subprocess.Popen(
        [sys.executable, str(TEST_NODES_DIR / script_name), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        time.sleep(1.0)
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=10.0)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)

    return process.returncode, output


def test_test_talker_exits_cleanly_on_sigint():
    return_code, output = _run_and_interrupt('test_talker.py')

    assert return_code == 0
    assert 'rcl_shutdown already called' not in output


def test_test_process_tree_exits_cleanly_on_sigint():
    return_code, output = _run_and_interrupt('test_process_tree.py')

    assert return_code == 0
    assert 'rcl_shutdown already called' not in output


def test_test_argument_talker_exits_cleanly_on_sigint():
    return_code, output = _run_and_interrupt('test_argument_talker.py', 'hello')

    assert return_code == 0
    assert 'rcl_shutdown already called' not in output