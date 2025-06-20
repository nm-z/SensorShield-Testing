import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUBS = ROOT / 'tests' / 'stubs'


def _run_cli(args, tmp_path):
    env = os.environ.copy()
    env['PYTHONPATH'] = f"{STUBS}{os.pathsep}" + env.get('PYTHONPATH', '')
    return subprocess.run(
        [sys.executable, str(ROOT / 'main.py'), *args],
        capture_output=True,
        text=True,
        cwd=str(ROOT),
        env=env,
    )


def _assert_output(file_path):
    assert file_path.exists(), 'Output file not created'
    with open(file_path, 'r', encoding='utf-8') as fh:
        data = json.load(fh)
    assert isinstance(data, list) and data, 'No data saved'
    keys = set(data[0].keys())
    assert 'temp' in keys and 'humidity' in keys
    assert keys <= {'temp', 'humidity', '_timestamp'}


def test_serial_cli_all_options(tmp_path):
    out_file = tmp_path / 'serial.json'
    result = _run_cli([
        '-sp',
        '--dump',
        '--addr', 'COM1',
        '-o', str(out_file),
        '--timeout', '1',
        '--json',
        '--filter', 'temp,humidity',
    ], tmp_path)
    assert result.returncode == 0
    _assert_output(out_file)


def test_wifi_cli_all_options(tmp_path):
    out_file = tmp_path / 'wifi.json'
    result = _run_cli([
        '-wf',
        '--dump',
        '--addr', '192.168.4.1',
        '-o', str(out_file),
        '--timeout', '1',
        '--json',
        '--filter', 'temp,humidity',
    ], tmp_path)
    assert result.returncode == 0
    _assert_output(out_file)


def test_bluetooth_cli_all_options(tmp_path):
    out_file = tmp_path / 'ble.json'
    result = _run_cli([
        '-bt',
        '--dump',
        '--addr', '00:11:22:33:44:55',
        '-o', str(out_file),
        '--timeout', '1',
        '--json',
        '--filter', 'temp,humidity',
    ], tmp_path)
    assert result.returncode == 0
    _assert_output(out_file)

