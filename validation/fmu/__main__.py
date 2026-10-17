"""Command line of the FMU parameter inventory: ``python -m validation.fmu --help``.

Needs Docker, the OpenModelica image of trano and FMPy. Set ``TRANO_MODELICA_LIBRARIES`` to a directory
of installed Modelica libraries to run without network access (see trano.simulate). The baseline FMU
takes as long to build as the model takes to compile in OpenModelica (tens of minutes for the six-zone
example); ``--baseline-fmu`` reuses one built earlier, and the results of the checks are stored in the
study directory so that a stopped study resumes where it was.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.fmu.calibration import InventoryBuilder, InventorySettings
from trano.fmu.checks import DAY, Window
from trano.fmu.export import FmuExportOptions, export_fmu
from validation.fmu.study import EQUIVALENCE_SAMPLE, run_study, write_outputs

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_YAML = ROOT / "examples" / "residential_after_renovation.yaml"
DEFAULT_DIRECTORY = ROOT / ".cache" / "fmu"
OUTPUT_DIRECTORY = Path(__file__).resolve().parent


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="python -m validation.fmu", description=__doc__)
    parser.add_argument("--yaml", type=Path, default=DEFAULT_YAML, help="building description")
    parser.add_argument("--library", default="Buildings", choices=["Buildings"], help="IDEAS is not resolved yet")
    parser.add_argument("--directory", type=Path, default=DEFAULT_DIRECTORY, help="FMUs, logs and stored results")
    parser.add_argument("--baseline-fmu", type=Path, help="reuse this baseline FMU instead of exporting it")
    parser.add_argument("--warm-up-days", type=float, default=7, help="simulated days discarded before the window")
    parser.add_argument("--days", type=float, default=14, help="length of the comparison window")
    parser.add_argument("--start-day", type=float, default=0, help="first simulated day of the year (0: 1 January)")
    parser.add_argument("--interval", type=float, default=600, help="[s] output interval")
    parser.add_argument(
        "--equivalence",
        nargs="*",
        help="YAML paths rebuilt for the equivalence check; the sample of one field per group when omitted, "
        "'none' for no rebuild",
    )
    parser.add_argument("--no-impact", action="store_true", help="skip the impact simulations")
    parser.add_argument("--output", type=Path, default=OUTPUT_DIRECTORY, help="where the CSV is written")
    arguments = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    directory = arguments.directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    library = Library.from_configuration(arguments.library)
    options = FmuExportOptions(library_name=library.name)
    fmu = arguments.baseline_fmu
    if fmu is None:
        baseline_directory = directory / "baseline"
        baseline_directory.mkdir(exist_ok=True)
        network = convert_network(arguments.yaml.stem, arguments.yaml.resolve(), library=library)
        fmu = export_fmu(baseline_directory, network, options=options, container_name="fmu_baseline")
    window = Window(
        start_time=arguments.start_day * DAY,
        warm_up=arguments.warm_up_days * DAY,
        duration=arguments.days * DAY,
        output_interval=arguments.interval,
    )
    settings = InventorySettings(
        yaml_path=arguments.yaml.resolve(),
        library_name=library.name,
        fmu=fmu.resolve(),
        work_directory=directory,
        window=window,
        export_options=options,
    )
    equivalence: tuple[str, ...] = EQUIVALENCE_SAMPLE
    if arguments.equivalence is not None:
        equivalence = () if arguments.equivalence == ["none"] else tuple(arguments.equivalence)
    inventory = run_study(
        InventoryBuilder(settings),
        directory / f"results_{library.name.lower()}.json",
        equivalence=equivalence,
        impact=not arguments.no_impact,
        rebuild_directory=directory / "rebuilds",
    )
    csv_path, meta_path = write_outputs(inventory, arguments.output)
    print(f"Inventory written to {csv_path} ({meta_path})")


if __name__ == "__main__":
    main()
