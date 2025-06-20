import json
import os
import subprocess
import sys
from pathlib import Path
import itertools
import pytest

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


DATA_SOURCES = {
    '-sp': 'COM1',
    '-wf': '192.168.4.1',
    '-bt': '00:11:22:33:44:55',
}

FLAG_COMBINATIONS = list(itertools.product([False, True], repeat=6))
FLAG_IDS = ["".join('1' if f else '0' for f in combo) for combo in FLAG_COMBINATIONS]


@pytest.mark.parametrize('source', DATA_SOURCES.keys(), ids=['sp', 'wf', 'bt'])
@pytest.mark.parametrize('combo', FLAG_COMBINATIONS, ids=FLAG_IDS)
def test_cli_all_combinations(source, combo, tmp_path):
    args = [source]
    out_file = None
    if combo[0]:
        args.append('--dump')
    if combo[1]:
        args.extend(['--addr', DATA_SOURCES[source]])
    if combo[2]:
        out_file = tmp_path / f"{source.strip('-')}_{''.join('1' if b else '0' for b in combo)}.json"
        args.extend(['-o', str(out_file)])
    if combo[3]:
        args.extend(['--timeout', '1'])
    if combo[4]:
        args.append('--json')
    if combo[5]:
        args.extend(['--filter', 'temp,humidity'])

    result = _run_cli(args, tmp_path)
    assert result.returncode == 0, f"Failed args {args}\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    if out_file:
        _assert_output(out_file)



