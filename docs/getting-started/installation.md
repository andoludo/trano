---
description: "Install Trano with pip, the Modelica libraries it generates models for, OpenModelica or its Docker image, and the optional Graphviz dependency used to lay out the model diagrams."
---
# Installation
## Optional dependencies

### Graphviz

Trano employs Graphviz to arrange various Modelica components within the model. While installation is not mandatory, it is advisable for enhanced model visualization. You can install Graphviz by following the instructions on the [official website](https://graphviz.org/download/). For Windows installations, ensure the Graphviz `bin` folder is added to the system PATH.

To install Graphviz on Linux, use the following command:

```bash
sudo apt-get install graphviz
```


```bash

sudo apt update
sudo apt install graphviz
                
```
            

### Docker

Trano utilizes the official OpenModelica Docker image to execute simulations. It is recommended to install Docker on your system so that Trano can pull the required images and run simulations. You can install Docker by following the instructions on the [official website](https://docs.docker.com/engine/install/).

### Modelica libraries

The simulation container installs the Modelica libraries Trano generates models for: Buildings 13.0.0 (built against Modelica 4.1.0), IDEAS 3.0.0 and AixLib 3.0.1 (both built against Modelica 4.0.0), on the `openmodelica/openmodelica:v1.26.9-ompython` image. Both Modelica Standard Library versions are installed side by side; the versions are defined in one place, `trano_simulate.ModelicaEnvironment`. IDEAS 4.0.0 is not used yet: no OpenModelica release handles it (1.25 cannot parse it, 1.26 and 1.27 fail with internal errors on its weather bus, also in IDEAS' own examples).

Each simulation container first runs an OpenModelica script that installs these libraries with the package manager. A library already present on the container's `MODELICAPATH` (for example baked into an image derived from the official one under `/root/.openmodelica/libraries`) is not installed again, so simulations also work where the package index cannot be reached.

Without further configuration, the libraries are installed at every container start, which needs network access. Two ways avoid it:

- mount a directory of installed libraries into every simulation container by setting the environment variable `TRANO_MODELICA_LIBRARIES` to it;
- install the libraries once, when an image is built, with the `omc` of that image: `python -m trano_simulate.install_libraries` runs the same installation as a container start and exits with a non-zero status when Modelica, Buildings, IDEAS or AixLib is still missing afterwards.

```dockerfile
FROM openmodelica/openmodelica:v1.26.9-ompython
RUN pip install trano && python -m trano_simulate.install_libraries
ENV TRANO_SIMULATION_RUNTIME=local
```

### Simulation runtimes

Trano runs OpenModelica in one of two ways, chosen by the `runtime` argument of `trano_simulate.simulate` or, by default, by the environment variable `TRANO_SIMULATION_RUNTIME`:

- `docker` (the default): a sibling container of the OpenModelica image is started through the Docker socket for every simulation, with a unique name, and the libraries are installed in it as described above;
- `local`: the `omc` found on the `PATH` runs in the process' machine, for example in a worker container built as above. Nothing is installed at run time; `trano.exceptions.OpenModelicaNotFoundError` is raised when `omc` is missing.

In both cases the model, its script and its weather file are written into the project directory and the result file lands in its `results` directory. Two simulations must not share a project directory.

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

`trano.check_yaml` checks the model with OpenModelica without simulating it, and `trano.kpi.building_kpis(result.result_path, network)` computes the heating energy, peak power and hours below the set point of models heated by `ideal_heating_cooling` emissions.

## Python package


!!! warning
    Trano requires python 3.9 or higher and docker to be installed on the system.
            

Trano is a Python package that you can install via pip.


```bash
pip install trano
```
            

Trano can also be utilized with Poetry.


```bash
poetry add trano
```
            

The Python package enables the generation of a Modelica model from the YAML configuration input. However, to simulate the model, Docker must be installed on the system.

On Windows systems, you may need to enable long path support. Refer to the [Microsoft documentation](https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=registry) for guidance.

## Check installation

To verify the installation, execute the following command in the terminal.


```bash
trano verify
```
            

This command checks your system's compatibility for model generation and simulation. Compatibility is optional if you only intend to generate the Modelica model.

