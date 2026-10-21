import tempfile
import webbrowser
from enum import Enum
from pathlib import Path
from typing import Annotated
from rich import print
import typer
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.mpc.parameters import RCModelType
from trano.reporting.html import to_html_reporting
from trano.reporting.reporting import ModelDocumentation
from trano.reporting.types import ResultFile
from trano.simulate.simulate import SimulationLibraryOptions, simulate
from trano.topology import Network
from trano.utils.utils import is_success
from rich.progress import Progress, SpinnerColumn, TextColumn

app = typer.Typer()
CHECKMARK = "[green]✔[/green]"
CROSS_MARK = "[red]✘[/red]"


class LibraryChoice(str, Enum):
    ideas = "IDEAS"
    buildings = "Buildings"
    mpc = "mpc"
    fast_model = "FastModel"
    fast_model_mpc = "FastModelMPC"


def _create_network(model: str, library: str, rc_model_type: RCModelType | None = None) -> Network:
    library_ = Library.from_configuration(library)
    if library_.is_rc and rc_model_type is not None:
        library_ = library_.model_copy(update={"rc_model_type": rc_model_type})
    model_ = Path(model).resolve()
    return convert_network(str(model_.stem), model_, library=library_)


def _simulation_result_path(model_path: Path) -> Path:
    return Path(model_path.parent) / "results" / f"{model_path.stem}.building_res.mat"


def _generate_report(network: Network, model_path: Path, result_path: Path) -> Path:
    reporting = ModelDocumentation.from_network(
        network,
        result=ResultFile(path=result_path),
    )
    html = to_html_reporting(reporting)
    report_path = Path(model_path.parent / f"{model_path.stem}.html")
    report_path.write_text(html)
    webbrowser.open(f"file://{report_path}")
    return report_path


@app.command()
def create_model(
    model: Annotated[
        str,
        typer.Argument(help="Local path to the '.yaml' model configuration file"),
    ],
    library: Annotated[
        LibraryChoice,
        typer.Argument(
            help="Library to be used: a simulation library, 'mpc' for a CasADi-compatible RC model, 'FastModel' "
            "for a closed-loop model close to Buildings that rumoca translates to CasADi, or 'FastModelMPC' for the "
            "same model with the controls of its systems as inputs, for optimal control."
        ),
    ] = LibraryChoice.buildings,
    rc_model_type: Annotated[
        RCModelType | None,
        typer.Option(help="RC zone model of the 'mpc' library (default: R3C2)."),
    ] = None,
    containers: Annotated[
        bool,
        typer.Option(
            help="Group the components in containers (envelope, emission, distribution, production, ...); "
            "--no-containers writes all of them in a single flat model."
        ),
    ] = True,
) -> None:
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        transient=True,
    ) as progress:
        modelica_model_path = Path(model).resolve().with_suffix(".mo")
        task = progress.add_task(
            description=f"Generating model {modelica_model_path.name} with library {library}",
            total=None,
        )
        network = _create_network(model, library, rc_model_type)
        modelica_model = network.model(include_container=containers)
        progress.update(task, completed=True)
        task = progress.add_task(description="Writing model to file...", total=None)
        modelica_model_path.write_text(modelica_model)
        progress.remove_task(task)
        print(f"{CHECKMARK} Model generated at {modelica_model_path}")
        if network.library.is_rc:
            from trano.mpc.modelica import network_interface

            interface_path = network_interface(network).write(modelica_model_path.with_suffix(".mpc.json"))
            print(f"{CHECKMARK} MPC interface of {network.name}.building_mpc generated at {interface_path}")


@app.command()
def simulate_model(
    model: Annotated[
        str,
        typer.Argument(help="Local path to the '.yaml' model configuration file."),
    ],
    library: Annotated[
        str | None,
        typer.Argument(help="Library to be used for simulation."),
    ] = LibraryChoice.buildings,
    start: Annotated[int | None, typer.Argument(help="Start simulation time.")] = 0,
    end: Annotated[int | None, typer.Argument(help="End simulation time.")] = 2 * 3600 * 24 * 7,
    tolerance: Annotated[float | None, typer.Argument(help="Simulation tolerance.")] = 1e-4,
) -> None:
    options = SimulationLibraryOptions(
        start_time=start,
        end_time=end,
        tolerance=tolerance,
        library_name=library,
    )
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        transient=True,
    ) as progress:
        task = progress.add_task(
            description=f"Generating model {Path(model).stem} with library {options.library_name}",
            total=None,
        )
        network = _create_network(model, options.library_name)
        model_ = Path(model).resolve()
        progress.remove_task(task)
        print(f"{CHECKMARK} Model generated successfully.")
        task = progress.add_task(
            description="Simulating model ...",
            total=None,
        )
        results = simulate(
            model_.parent,
            network,
            options=options,
        )

        if not is_success(results):
            print(f"{CROSS_MARK} Simulation failed. See logs for more information.")
            return

        result_path = _simulation_result_path(model_)
        if not result_path.exists():
            print(f"{CROSS_MARK} Simulation failed. Result file not found in {result_path}.")
            return
        progress.remove_task(task)
        print(f"{CHECKMARK} Simulation results available at {result_path}")

        task = progress.add_task(
            description="Creating report ...",
            total=None,
        )
        report_path = _generate_report(network, model_, result_path)
        progress.remove_task(task)
        print(f"{CHECKMARK} Report available at {report_path}")


def _parse_assignments(assignments: list[str] | None, option: str) -> dict[str, float]:
    parsed = {}
    for assignment in assignments or []:
        name, separator, value = assignment.partition("=")
        if not separator or not name.strip():
            raise typer.BadParameter(f"expected name=value, got {assignment!r}", param_hint=option)
        try:
            parsed[name.strip()] = float(value)
        except ValueError as error:
            raise typer.BadParameter(f"{value!r} is not a number", param_hint=option) from error
    return parsed


@app.command()
def compile_model(  # noqa: PLR0913
    model: Annotated[str, typer.Argument(help="Local path to the '.yaml' model configuration file.")],
    library: Annotated[LibraryChoice, typer.Argument(help="Library to be used.")] = LibraryChoice.buildings,
    output: Annotated[
        Path | None,
        typer.Option(help="Project directory of the build; '<model>_runtime' next to the YAML file by default."),
    ] = None,
    start: Annotated[int, typer.Option(help="Default start time of the runs (s).")] = 0,
    end: Annotated[int, typer.Option(help="Default stop time of the runs (s).")] = 2 * 3600 * 24 * 7,
    tolerance: Annotated[float, typer.Option(help="Simulation tolerance.")] = 1e-4,
    intervals: Annotated[int | None, typer.Option(help="Number of output intervals.")] = None,
) -> None:
    """Compile the model once; run it many times with 'trano run-model' and new parameter values."""
    from trano.runtime import build_model

    model_path = Path(model).resolve()
    project = (output or model_path.parent / f"{model_path.stem}_runtime").resolve()
    options = SimulationLibraryOptions(
        start_time=start, end_time=end, tolerance=tolerance, number_of_intervals=intervals, library_name=library
    )
    with Progress(SpinnerColumn(), TextColumn("[progress.description]{task.description}"), transient=True) as progress:
        progress.add_task(description=f"Compiling {model_path.stem} with library {library.value}", total=None)
        network = _create_network(model, library)
        compiled = build_model(project, network, options, specification=model_path)
    print(f"{CHECKMARK} Executable and {compiled.init_xml.name} kept in {compiled.directory}")


@app.command()
def run_model(  # noqa: PLR0913
    build: Annotated[Path, typer.Argument(help="Build directory written by 'trano compile-model'.")],
    set_: Annotated[
        list[str] | None,
        typer.Option("--set", help="Run-time parameter override 'name=value' (repeatable)."),
    ] = None,
    change: Annotated[
        list[str] | None,
        typer.Option(
            help="Change of the YAML specification 'path=value' (repeatable), e.g. "
            "'material[EPS:001].thermal_conductivity=0.035', translated into run-time overrides."
        ),
    ] = None,
    start: Annotated[float | None, typer.Option(help="Start time (s).")] = None,
    stop: Annotated[float | None, typer.Option(help="Stop time (s).")] = None,
    label: Annotated[str, typer.Option(help="Name of the run: runs/<label>_result.mat.")] = "run",
    strict: Annotated[bool, typer.Option(help="Fail when the executable ignores an override.")] = True,
) -> None:
    """Run a compiled model with new parameter values, without recompiling."""
    from trano.runtime import CompiledModel, run_model as run_compiled_model
    from trano.runtime.resolution import NotRuntimeChangeableError, SpecificationResolver

    compiled = CompiledModel.load(build)
    overrides = _parse_assignments(set_, "--set")
    changes = _parse_assignments(change, "--change")
    if changes:
        if compiled.specification is None or compiled.library is None:
            raise typer.BadParameter("the build does not record its YAML specification", param_hint="--change")
        resolver = SpecificationResolver.from_file(Path(compiled.specification), compiled.library)
        try:
            resolved = resolver.overrides(dict(changes), compiled)
        except NotRuntimeChangeableError as error:
            if strict:
                raise typer.BadParameter(str(error), param_hint="--change") from error
            print(f"[yellow]warning[/yellow] {error}; only the free parameters are set")
            resolved = error.free
        overrides = resolved | overrides
    with Progress(SpinnerColumn(), TextColumn("[progress.description]{task.description}"), transient=True) as progress:
        progress.add_task(description=f"Running {compiled.model_name} ({len(overrides)} overrides)", total=None)
        result = run_compiled_model(compiled, overrides, label=label, start_time=start, stop_time=stop, strict=strict)
    for message in result.messages:
        print(f"[yellow]warning[/yellow] {message.name}: {message.issue}")
    print(f"{CHECKMARK} Results available at {result.result_path}")


@app.command()
def parameter_inventory(  # noqa: PLR0913
    model: Annotated[str, typer.Argument(help="Local path to the '.yaml' model configuration file.")],
    library: Annotated[LibraryChoice, typer.Argument(help="Library to be used.")] = LibraryChoice.buildings,
    project: Annotated[
        Path | None, typer.Option(help="Working directory; '<model>_inventory' next to the YAML file by default.")
    ] = None,
    checks: Annotated[
        bool, typer.Option(help="Run the readback, impact and equivalence checks (hours on a large model).")
    ] = True,
    select: Annotated[str | None, typer.Option(help="Regular expression of the YAML paths to study.")] = None,
    workers: Annotated[int, typer.Option(help="Simulations run at the same time.")] = 3,
) -> None:
    """Which YAML fields can be changed at run time in the compiled model (parameter_inventory_<library>.csv)."""
    from trano.runtime.study import InventoryStudy

    model_path = Path(model).resolve()
    project_path = project or model_path.parent / f"{model_path.stem}_inventory"
    study = InventoryStudy(model_path, library.value, project_path, workers=workers, select=select)
    rows = study.run(checks=checks)
    print(f"{CHECKMARK} {len(rows)} fields in {project_path / f'parameter_inventory_{library.value.lower()}.csv'}")


@app.command()
def verify() -> None:
    model_path = Path(__file__).parent / "verification.yaml"
    options = SimulationLibraryOptions(end_time=3600)
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        transient=True,
    ) as progress:
        task = progress.add_task(
            description="Verify generating model...",
            total=None,
        )
        network = _create_network(str(model_path), options.library_name)

        progress.remove_task(
            task,
        )
        print(f"{CHECKMARK} Model generated successfully. Your system is compatible for model generation.")
        task_ = progress.add_task(
            description="Verify simulation...",
            total=None,
        )
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
            try:
                simulate(
                    Path(project_path),
                    network,
                    options=options,
                )
            except Exception as e:
                print(f"{CROSS_MARK} Simulation failed. Reason {e}.")
                return
        progress.remove_task(
            task_,
        )
        print(f"{CHECKMARK} Model simulated successfully. Your system is compatible for simulation.")


def report(model: Path | str, options: SimulationLibraryOptions) -> None:
    model = Path(model).resolve()
    network = convert_network(model.stem, model, library=Library.from_configuration(options.library_name))
    _generate_report(network, model, _simulation_result_path(model))


if __name__ == "__main__":
    app()
