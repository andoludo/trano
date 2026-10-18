"""A stand-in for OpenModelica's ``omc``, to test the local runtime without OpenModelica.

It replays a stored OpenModelica log (``tests/data/omc_logs``) chosen by ``FAKE_OMC_LOG``, writes the result
file of the model checked by the script into its working directory, as ``omc`` does, and records its
arguments, working directory and script in ``FAKE_OMC_RECORD``. ``FAKE_OMC_SLEEP`` makes it, and a child process
it starts, hang that long.
"""

import json
import os
import stat
import sys
from collections.abc import Iterator
from pathlib import Path

import pytest

from tests.fixtures.paths import TESTS_DIRECTORY

OMC_LOGS = TESTS_DIRECTORY / "data" / "omc_logs"

FAKE_OMC = """
import json, os, re, subprocess, sys, time
from pathlib import Path

script = Path(sys.argv[1]).read_text()
Path(os.environ["FAKE_OMC_RECORD"]).write_text(
    json.dumps({"argv": sys.argv[1:], "cwd": os.getcwd(), "script": script})
)
print("fake omc started", flush=True)
sleep_s = float(os.environ.get("FAKE_OMC_SLEEP", "0"))
if sleep_s:  # like the simulation executable omc starts, a child holding the output open
    subprocess.Popen([sys.executable, "-c", f"import time; time.sleep({sleep_s})"])
time.sleep(sleep_s)
log = Path(os.environ["FAKE_OMC_LOG"]).read_text()
sys.stdout.write(log)
if "The simulation finished successfully" in log:
    model = re.search(r"checkModel\\((\\w+)\\.building\\)", script).group(1)
    Path(f"{model}.building_res.mat").write_bytes(b"")
"""


class FakeOmc:
    def __init__(self, directory: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        self.record_path = directory / "record.json"
        self._monkeypatch = monkeypatch
        self.replay("success.log")

    def replay(self, log: str, sleep_s: float = 0) -> None:
        self._monkeypatch.setenv("FAKE_OMC_LOG", str(OMC_LOGS / log))
        self._monkeypatch.setenv("FAKE_OMC_SLEEP", str(sleep_s))

    @property
    def record(self) -> dict[str, object]:
        return dict(json.loads(self.record_path.read_text()))


@pytest.fixture
def fake_omc(tmp_path_factory: pytest.TempPathFactory, monkeypatch: pytest.MonkeyPatch) -> Iterator[FakeOmc]:
    directory = tmp_path_factory.mktemp("fake_omc")
    executable = directory / "omc"
    executable.write_text(f"#!{sys.executable}\n{FAKE_OMC}")
    executable.chmod(executable.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv("PATH", f"{directory}{os.pathsep}{os.environ.get('PATH', '')}")
    monkeypatch.setenv("FAKE_OMC_RECORD", str(directory / "record.json"))
    yield FakeOmc(directory, monkeypatch)
