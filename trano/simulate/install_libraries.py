"""Install the Modelica libraries of the OpenModelica environment with the ``omc`` of this machine.

Meant for an image build, so that containers of the image start in seconds and need no network::

    RUN python -m trano.simulate.install_libraries

It runs the sequence the docker runtime runs at every container start (``configure.mos``, the SDF
extraction, ``configure.mos`` again) and exits with a non-zero status when a library is still missing.
"""

import logging
import re
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Sequence
from pathlib import Path

from trano.simulate.simulate import EXTRACT_SDF, MODELICA_ENVIRONMENT, ModelicaEnvironment

logger = logging.getLogger(__name__)

REQUIRED_LIBRARIES = ("Modelica", "Buildings", "IDEAS", "AixLib")
OPENMODELICA_HOME = Path.home() / ".openmodelica"  # where OpenModelica's package manager installs, per user


def _run(command: Sequence[str], cwd: Path) -> str:
    completed = subprocess.run(command, cwd=cwd, capture_output=True, text=True, check=False)  # noqa: S603
    output = completed.stdout + completed.stderr
    logger.debug("%s exited with %s:\n%s", " ".join(command), completed.returncode, output)
    return output


def available_libraries(omc: str, work_dir: Path) -> set[str]:
    """The libraries OpenModelica finds on its MODELICAPATH."""
    script = work_dir / "available_libraries.mos"
    script.write_text("getAvailableLibraries();\n")
    output = _run([omc, str(script)], cwd=work_dir)
    listing = next((line for line in reversed(output.splitlines()) if line.startswith("{")), "{}")
    return set(re.findall(r'"([^"]+)"', listing))


def install_libraries(
    environment: ModelicaEnvironment = MODELICA_ENVIRONMENT,
    omc: str = "omc",
    openmodelica_home: Path = OPENMODELICA_HOME,
) -> list[str]:
    """Install the libraries of ``environment``; the required libraries still missing afterwards."""
    with tempfile.TemporaryDirectory(prefix="trano-libraries-") as directory:
        work_dir = Path(directory)
        configure = work_dir / "configure.mos"
        configure.write_text(environment.configure_script())
        extract_sdf = work_dir / "extract_sdf.py"
        extract_sdf.write_text(EXTRACT_SDF)
        _run([omc, str(configure)], cwd=work_dir)
        # AixLib fails to install on its SDF dependency: install SDF by hand, then configure again,
        # which installs what is still missing and only checks the rest.
        cache, libraries = openmodelica_home / "cache", openmodelica_home / "libraries"
        _run([sys.executable, str(extract_sdf), str(cache), str(libraries), environment.sdf], cwd=work_dir)
        _run([omc, str(configure)], cwd=work_dir)
        available = available_libraries(omc, work_dir)
    return [library for library in REQUIRED_LIBRARIES if library not in available]


def main() -> int:
    omc = shutil.which("omc")
    if omc is None:
        print("OpenModelica's omc is not on the PATH: the Modelica libraries cannot be installed.", file=sys.stderr)
        return 1
    missing = install_libraries(omc=omc)
    if missing:
        print(f"Modelica libraries missing after the installation: {', '.join(missing)}.", file=sys.stderr)
        return 1
    print(f"Modelica libraries installed: {', '.join(REQUIRED_LIBRARIES)}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
