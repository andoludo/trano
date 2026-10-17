# Export a model as an FMU

trano exports the generated model as an FMI 2.0 co-simulation FMU, built by OpenModelica in the same
container image as a simulation, so that the FMU and the OpenModelica run of a model share one compiler,
one library installation and one solver family.

```bash
trano export-fmu examples/residential_after_renovation.yaml Buildings
```

writes `examples/residential_after_renovation.fmu` next to the YAML (and a `.fmu.log` with the
OpenModelica output). From Python:

```python
from pathlib import Path

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.fmu import export_fmu

yaml = Path("examples/residential_after_renovation.yaml")
network = convert_network(yaml.stem, yaml, library=Library.from_configuration("Buildings"))
fmu = export_fmu(yaml.parent, network)
```

The FMU simulates with any FMI 2.0 importer; with [FMPy](https://github.com/CATIA-Systems/FMPy):

```python
from fmpy import simulate_fmu

result = simulate_fmu(
    "residential_after_renovation.fmu",
    start_time=0,
    stop_time=7 * 24 * 3600,
    output_interval=600,
    start_values={"building.emission1.emission_control_living_001.THeaSet": 295.15},
    output=["building.bus1.dataBus.TZonLiving_001", "building.bus1.dataBus.heat_flowGenerator_001"],
)
```

## What is exported, and why

- **Co-simulation with CVODE.** The FMU carries its own solver (`--fmiFlags=s:cvode`): every importer
  integrates the building with the same stiff solver and event handling as OpenModelica, instead of the
  default explicit Euler of co-simulation FMUs or the importer's own solver of a model-exchange FMU. The
  type, the FMI version, the solver and the image are the constants of `trano.fmu.export`
  (`FmuExportOptions` overrides them per call; `--fmu-type me` on the command line exports a
  model-exchange FMU).
- **A `static` linux64 binary.** The image produces Linux binaries: simulate the FMU on Linux, or
  inside the container on another platform.
- **The weather file travels with the FMU.** The data files a model reads (`modelica://` resources such
  as the Uccle weather of IDEAS used by every library by default) are packed into the `resources/`
  directory of the FMU and read from there, not from a path of the machine that built it. The FMU
  tests simulate from a scratch directory to check this.
- **Parameters.** OpenModelica exports the parameters of the model with `causality = parameter` and
  `variability = fixed`: they are set before initialisation, which is what a calibration does when it
  re-instantiates the FMU at every run. The [FMU parameter inventory](../validation/fmu_parameters.md)
  lists, for each field of the YAML, the FMU parameter it ends up in and whether setting that parameter
  is the same as changing the YAML.

## Inventory of the calibration parameters

```bash
python -m validation.fmu --help
```

builds the baseline FMU of `examples/residential_after_renovation.yaml`, resolves every numeric field of
the YAML to the FMU parameters it sets and runs the readback, impact and equivalence checks described in
the [inventory report](../validation/fmu_parameters.md). `pytest -m fmu` runs a short version of the same
checks (Docker, the OpenModelica image and FMPy are needed; `TRANO_FMU_BASELINE=<file>` reuses a built
FMU).
