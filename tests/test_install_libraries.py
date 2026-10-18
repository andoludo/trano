"""The installation of the Modelica libraries at image build time."""

import os
import stat
import sys
from pathlib import Path

import pytest

from trano.simulate import install_libraries as installer

# Records the scripts it runs and answers ``getAvailableLibraries()`` with FAKE_LIBRARIES.
FAKE_OMC = """
import os, sys
from pathlib import Path

script = Path(sys.argv[1])
with open(os.environ["FAKE_OMC_CALLS"], "a") as calls:
    calls.write(script.name + "\\n")
if "getAvailableLibraries" in script.read_text():
    print('"OpenModelica 1.26.9"')
    print("{" + ", ".join(f'"{name}"' for name in os.environ["FAKE_LIBRARIES"].split(",")) + "}")
"""


@pytest.fixture
def omc(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    executable = tmp_path / "bin" / "omc"
    executable.parent.mkdir()
    executable.write_text(f"#!{sys.executable}\n{FAKE_OMC}")
    executable.chmod(executable.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv("PATH", f"{executable.parent}{os.pathsep}{os.environ.get('PATH', '')}")
    monkeypatch.setenv("FAKE_OMC_CALLS", str(tmp_path / "calls.txt"))
    monkeypatch.setattr(installer, "OPENMODELICA_HOME", tmp_path / ".openmodelica")
    return tmp_path / "calls.txt"


def test_the_libraries_are_configured_twice_around_the_sdf_extraction(
    omc: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setenv("FAKE_LIBRARIES", "Modelica,ModelicaServices,Complex,Buildings,IDEAS,AixLib,SDF")

    assert installer.main() == 0

    assert omc.read_text().splitlines() == ["configure.mos", "configure.mos", "available_libraries.mos"]
    assert "AixLib" in capsys.readouterr().out


def test_a_missing_library_fails_the_installation(
    omc: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setenv("FAKE_LIBRARIES", "Modelica,Buildings,IDEAS")

    assert installer.main() == 1

    assert "missing after the installation: AixLib" in capsys.readouterr().err


def test_the_installation_needs_omc(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("PATH", str(tmp_path))

    assert installer.main() == 1
