from pathlib import Path

from docs_src.utils import CleanedText, DisplayObject, TitleText, Tutorial, Text

INSTALLATIONS = [
    Tutorial(
        title="Installation",
        description=(
            "Install Trano with pip, the Modelica libraries it generates models for, OpenModelica or its Docker "
            "image, and the optional Graphviz dependency used to lay out the model diagrams."
        ),
        contents=[
            TitleText(content="""Optional dependencies"""),
            CleanedText(
                content="""### Graphviz
                Trano uses graphviz to position the different modelica components in the model.
                Although it is not necessary, it is recommended to install graphviz to have a better visualization 
                of the model.
                Graphviz can be installed by following the instructions on the official 
                website(https://graphviz.org/download/).
                For windows install, make sure that the graphviz bin folder is added to the system path."""
            ),
            CleanedText(content="""For linux, one can use the following command to install graphviz."""),
            DisplayObject(
                language="bash",
                object="""
sudo apt update
sudo apt install graphviz
                """,
            ),
            CleanedText(
                content="""### Docker
    Similarly, trano uses the official openModelica docker image to run simulations. Therefore, it is recommended 
    to install docker on the system so
    trano can pull the necessary images and run simulations. One can install Docker by following the instructions 
    on the official
                website(https://docs.docker.com/engine/install/)."""
            ),
            CleanedText(
                content="""### Modelica libraries
    The simulation container installs the Modelica libraries Trano generates models for: Buildings 13.0.0
    (built against Modelica 4.1.0), IDEAS 3.0.0 and AixLib 3.0.1 (both built against Modelica 4.0.0), on the
    openmodelica/openmodelica:v1.26.9-ompython image. Both Modelica Standard Library versions are installed
    side by side; the versions are defined in one place, trano_simulate.ModelicaEnvironment.
    Each simulation container first runs an OpenModelica script that installs these libraries with the
    package manager. A library already present on the container's MODELICAPATH (for example baked into an
    image derived from the official one under /root/.openmodelica/libraries) is not installed again, so
    simulations also work where the package index cannot be reached."""
            ),
            Text(
                content="""
Without further configuration, the libraries are installed at every container start, which needs network
access. Two ways avoid it:

- mount a directory of installed libraries into every simulation container by setting the environment variable
  `TRANO_MODELICA_LIBRARIES` to it;
- install the libraries once, when an image is built, with the `omc` of that image: `python -m
  trano_simulate.install_libraries` runs the same installation as a container start and exits with a non-zero
  status when Modelica, Buildings, IDEAS or AixLib is still missing afterwards.

```dockerfile
FROM openmodelica/openmodelica:v1.26.9-ompython
RUN pip install trano && python -m trano_simulate.install_libraries
ENV TRANO_SIMULATION_RUNTIME=local
```

### Simulation runtimes

Trano runs OpenModelica in one of two ways, chosen by the `runtime` argument of
`trano_simulate.simulate` or, by default, by the environment variable `TRANO_SIMULATION_RUNTIME`:

- `docker` (the default): a sibling container of the OpenModelica image is started through the Docker socket
  for every simulation, with a unique name, and the libraries are installed in it as described above;
- `local`: the `omc` found on the `PATH` runs in the process' machine, for example in a worker container built
  as above. Nothing is installed at run time; `trano.exceptions.OpenModelicaNotFoundError` is raised when
  `omc` is missing.

In both cases the model, its script and its weather file are written into the project directory and the result
file lands in its `results` directory. Two simulations must not share a project directory.

The one-call API validates, generates and simulates a model and returns a typed result instead of raising:

```python
from pathlib import Path

from trano import simulate_yaml
from trano_simulate.simulate import SimulationOptions

result = simulate_yaml(
    Path("house.yaml"),  # or the model already parsed into a dict
    options=SimulationOptions(end_time=7 * 24 * 3600),
    runtime="local",
    on_phase=print,  # generating, compiling, simulating, done
)
if result.success:
    print(result.result_path)
else:
    print(result.error_kind, result.error_message)  # validation, translation, solver, timeout or runtime
```

`trano.check_yaml` checks the model with OpenModelica without simulating it, and
`trano.kpi.building_kpis(result.result_path, network)` computes the heating energy, peak power and hours below
the set point of models heated by `ideal_heating_cooling` emissions."""
            ),
            TitleText(content="""Python package"""),
            Text(
                content="""
!!! warning
    Trano requires python 3.9 or higher and docker to be installed on the system.
            """
            ),
            CleanedText(content="""Trano is a python package that can be installed using pip."""),
            DisplayObject(language="bash", object="pip install trano"),
            CleanedText(content="""Trano can also be used using poetry."""),
            DisplayObject(language="bash", object="poetry add trano"),
            CleanedText(
                content="""The python package will allow to generate the Modelica model from the yaml
                configuration input.
                However, in order to be able to simulate model, Docker needs to be installed on the system."""
            ),
            CleanedText(
                content="""On windows systems, you may need to enable long path supports 
                (see https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=registry)"""
            ),
            TitleText(content="""Check installation"""),
            CleanedText(content="""To check installation, run the following command in the terminal."""),
            DisplayObject(
                language="bash",
                object="""trano verify""",
            ),
            CleanedText(
                content="""This command will check if your system is compatible for model generation and simulation.
                Being compatible for model simulation is optional if you only want to generate the Modelica model
                 only."""
            ),
        ],
    )
]


def gen_installation_docs() -> None:
    for installation in INSTALLATIONS:
        installation.write(Path(__file__).parents[2].joinpath("docs", "getting-started", f"{installation.name()}.md"))


if __name__ == "__main__":
    gen_installation_docs()
