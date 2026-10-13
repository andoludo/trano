import tempfile
import webbrowser
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Annotated
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

if TYPE_CHECKING:
    from trano.mpc.interface import MPCModelInterface

app = typer.Typer()
CHECKMARK = "[green]✔[/green]"
CROSS_MARK = "[red]✘[/red]"


class LibraryChoice(str, Enum):
    ideas = "IDEAS"
    buildings = "Buildings"
    mpc = "mpc"
    fast_model = "FastModel"


class Envelope(str, Enum):
    rc = "rc"
    multilayer = "multilayer"


def _create_network(
    model: str, library: str, rc_model_type: RCModelType | None = None, envelope: Envelope | None = None
) -> Network:
    library_ = Library.from_configuration(library)
    if library_.is_rc and rc_model_type is not None:
        library_ = library_.model_copy(update={"rc_model_type": rc_model_type})
    if envelope is not None:
        if not library_.is_fast_model:
            raise typer.BadParameter("--envelope only applies to the 'FastModel' library.")
        library_ = library_.model_copy(update={"envelope": envelope.value})
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


def _network_interface(network: Network) -> "MPCModelInterface":
    if network.library.is_fast_model:
        from trano.fast_model.modelica import network_interface as fast_model_interface

        return fast_model_interface(network)
    from trano.mpc.modelica import network_interface

    return network_interface(network)


@app.command()
def create_model(
    model: Annotated[
        str,
        typer.Argument(help="Local path to the '.yaml' model configuration file"),
    ],
    library: Annotated[
        LibraryChoice,
        typer.Argument(
            help="Library to be used: a simulation library, 'mpc' for a CasADi-compatible RC model or 'FastModel' "
            "for a CasADi-compatible model close to Buildings, with its hydronic circuits."
        ),
    ] = LibraryChoice.buildings,
    rc_model_type: Annotated[
        RCModelType | None,
        typer.Option(help="RC zone model of the 'mpc' library, and of the 'FastModel' rc envelope (default: R3C2)."),
    ] = None,
    envelope: Annotated[
        Envelope | None,
        typer.Option(
            help="'FastModel' library: 'multilayer' zones with the physics of the Buildings MixedAir zone (default) "
            "or 'rc' zones."
        ),
    ] = None,
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
        network = _create_network(model, library, rc_model_type, envelope)
        modelica_model = network.model()
        progress.update(task, completed=True)
        task = progress.add_task(description="Writing model to file...", total=None)
        modelica_model_path.write_text(modelica_model)
        progress.remove_task(task)
        print(f"{CHECKMARK} Model generated at {modelica_model_path}")
        if network.library.is_rc:
            interface_path = _network_interface(network).write(modelica_model_path.with_suffix(".mpc.json"))
            print(f"{CHECKMARK} MPC interface of {network.name}.building_mpc generated at {interface_path}")


@app.command()
def simulate_model(  # noqa: PLR0913 - one argument per CLI option
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
    envelope: Annotated[
        Envelope | None,
        typer.Option(help="'FastModel' library: 'multilayer' zones (default) or 'rc' zones."),
    ] = None,
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
        network = _create_network(model, options.library_name, envelope=envelope)
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
