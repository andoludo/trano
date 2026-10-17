"""Export a network as an FMU with OpenModelica's ``buildModelFMU`` in trano's simulation container.

The FMU is a co-simulation FMU (``FMU_TYPE``): it carries its own solver, so an importer only steps
it and every importer (FMPy, a calibration loop, a co-simulation master) gets the same numerics
as the OpenModelica run of the same model. A model-exchange FMU would leave the integration of
a stiff, event-rich Buildings model (valves, hysteresis, schedules) to the importer's solver, whose
tolerances and event handling then differ from tool to tool. OpenModelica's default co-simulation
solver is explicit Euler, which cannot integrate a building model with hydronics at any useful
step: the FMU is built with CVODE (``FMU_SOLVER``), which the OpenModelica FMI runtime embeds when
``--fmiFlags=s:cvode`` is set.

The Docker and OpenModelica calls go through :mod:`trano.simulate.simulate`: the same image, the same
library installation and the same ``/simulation`` mount as a simulation.
"""

import logging
from collections.abc import Callable, Generator
from contextlib import contextmanager
from pathlib import Path
import tempfile

from pydantic import BaseModel, Field

from trano.elements.jinja import STRING_ENVIRONMENT
from trano.exceptions import FmuError
from trano.simulate.simulate import (
    MODELICA_ENVIRONMENT,
    ModelicaEnvironment,
    load_aixlib,
    client,
    container,
)
from trano.topology import Network

logger = logging.getLogger(__name__)

FMI_VERSION = "2.0"
"""FMI standard version written by ``buildModelFMU``."""
FMU_TYPE = "cs"
"""``cs`` (co-simulation, with the solver below), ``me`` (model exchange) or ``me_cs`` (both)."""
FMU_SOLVER = "cvode"
"""Solver embedded in a co-simulation FMU (``--fmiFlags=s:<solver>``): ``cvode`` or ``euler``."""
FMU_PLATFORMS = ("static",)
"""Platforms of ``buildModelFMU``: ``static`` links the runtime into the binary of the build machine (linux64)."""
FMU_IMAGE = MODELICA_ENVIRONMENT.openmodelica_image
"""OpenModelica image building the FMU: the simulation image, so that both runs share one compiler."""
FMU_CONTAINER_NAME = "openmodelica_fmu"
"""Name of the container; distinct from the simulation container so that both can run at once."""

ScriptBuilder = Callable[[str], str]


class FmuExportOptions(BaseModel):
    """What ``buildModelFMU`` is called with; the module constants are the defaults."""

    library_name: str = Field(default="Buildings", description="Library the network was generated with")
    fmi_version: str = Field(default=FMI_VERSION)
    fmu_type: str = Field(default=FMU_TYPE, pattern="^(cs|me|me_cs)$")
    solver: str = Field(default=FMU_SOLVER, description="Co-simulation solver; ignored by a model-exchange FMU")
    platforms: tuple[str, ...] = Field(default=FMU_PLATFORMS)
    include_resources: bool = Field(
        default=True, description="Pack the ``Resources`` of the model package into the FMU (``resources/``)"
    )
    image: str = Field(default=FMU_IMAGE)

    @property
    def command_line_options(self) -> list[str]:
        """``setCommandLineOptions`` statements preceding the build."""
        options = []
        if "cs" in self.fmu_type:
            options.append(f"--fmiFlags=s:{self.solver}")
        return options

    def build_statement(self, model: str, file_name_prefix: str) -> str:
        platforms = ", ".join(f'"{platform}"' for platform in self.platforms)
        return (
            f'buildModelFMU({model}, version="{self.fmi_version}", fmuType="{self.fmu_type}", '
            f'fileNamePrefix="{file_name_prefix}", platforms={{{platforms}}}, '
            f"includeResources={str(self.include_resources).lower()});"
        )


def fmu_file_name(network: Network) -> str:
    return f"{network.name}.fmu"


def fmu_script(
    network: Network,
    model_file: str,
    options: FmuExportOptions,
    model: str,
    environment: ModelicaEnvironment = MODELICA_ENVIRONMENT,
) -> str:
    """The OpenModelica script building the FMU of ``network.building`` from ``/simulation/<model_file>``.

    The libraries are loaded as for a simulation (:func:`trano.simulate.simulate.create_mos_file`):
    the Modelica Standard Library, IDEAS (whose weather files are the default of every library) and
    AixLib when the model uses it; Buildings is loaded through the ``uses`` annotation of the package.
    """
    statements = [
        "getVersion();",
        f'loadModel(Modelica, {{"{environment.modelica_version}"}});',
        f'loadModel(IDEAS, {{"{environment.ideas}"}});' + load_aixlib(model, environment),
        f'loadFile("/simulation/{model_file}"); getErrorString();',
        'cd("/simulation");',
        *(f'setCommandLineOptions("{option}"); getErrorString();' for option in options.command_line_options),
        options.build_statement(f"{network.name}.building", network.name),
        "getErrorString();",
    ]
    return STRING_ENVIRONMENT.from_string("\n".join(statements)).render()


@contextmanager
def _script_files(project_path: Path, model: str, script_of: ScriptBuilder) -> Generator[str, None, None]:
    with (
        tempfile.NamedTemporaryFile(mode="w", dir=project_path, suffix=".mo") as model_file,
        tempfile.NamedTemporaryFile(mode="w", dir=project_path, suffix=".mos") as mos_file,
    ):
        Path(model_file.name).write_text(model)
        Path(mos_file.name).write_text(script_of(Path(model_file.name).name))
        yield Path(mos_file.name).name


def export_fmu(
    project_path: Path,
    network: Network,
    options: FmuExportOptions | None = None,
    container_name: str = FMU_CONTAINER_NAME,
    model: str | None = None,
) -> Path:
    """Build the FMU of the network in ``project_path`` and return its path.

    ``model`` is the Modelica text of the network when it was rendered already (a network renders
    its model once). The weather files given as local paths are copied next to the model, as for
    a simulation; a ``modelica://`` resource is read from the library installed in the container.
    Raises :class:`FmuError` with the OpenModelica output when no FMU comes out.
    """
    options = options or FmuExportOptions()
    project_path = project_path.resolve()
    project_path.mkdir(parents=True, exist_ok=True)
    if model is None:
        network.set_weather_path_to_container_path(project_path)
        model = network.model()
    environment = MODELICA_ENVIRONMENT.model_copy(update={"openmodelica_image": options.image})
    fmu_path = project_path / fmu_file_name(network)
    fmu_path.unlink(missing_ok=True)

    def script_of(model_file: str) -> str:
        return fmu_script(network, model_file, options, model, environment)

    with (
        container(client(), project_path, environment=environment, container_name=container_name) as container_,
        _script_files(project_path, model, script_of) as mos_file,
    ):
        result = container_.exec_run(cmd=f"omc /simulation/{mos_file}")
    output = result.output.decode(errors="replace")
    (project_path / f"{network.name}.fmu.log").write_text(output)
    if not fmu_path.exists():
        raise FmuError(f"OpenModelica did not build {fmu_path.name}:\n{output[-4000:]}")
    logger.info("FMU built at %s", fmu_path)
    return fmu_path
