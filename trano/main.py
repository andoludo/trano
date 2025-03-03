import tempfile
import webbrowser
from enum import Enum
from pathlib import Path
from typing import Optional, Annotated
from rich import print
import typer
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
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


def _create_network(model: str, library: str) -> Network:
    library_ = Library.from_configuration(library)
    model_ = Path(model).resolve()
    return convert_network(str(model_.stem), model_, library=library_)


@app.command()
def create_model(
    model: Annotated[
        str,
        typer.Argument(help="Local path to the '.yaml' model configuration file"),
    ],
    library: Annotated[
        LibraryChoice,
        typer.Argument(help="Library to be used for simulation."),
    ] = LibraryChoice.buildings,
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
        network = _create_network(model, library)
        modelica_model = network.model()
        progress.update(task, completed=True)
        task = progress.add_task(description="Writing model to file...", total=None)
        modelica_model_path.write_text(modelica_model)
        progress.remove_task(task)
        print(f"{CHECKMARK} Model generated at {modelica_model_path}")


@app.command()
def simulate_model(
    model: Annotated[
        str,
        typer.Argument(help="Local path to the '.yaml' model configuration file."),
    ],
    library: Annotated[
        Optional[str],
        typer.Argument(help="Library to be used for simulation."),
    ] = LibraryChoice.buildings,
    start: Annotated[Optional[int], typer.Argument(help="Start simulation time.")] = 0,
    end: Annotated[Optional[int], typer.Argument(help="End simulation time.")] = 2
    * 3600
    * 24
    * 7,
    tolerance: Annotated[
        Optional[float], typer.Argument(help="Simulation tolerance.")
    ] = 1e-4,
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

        result_path = (
            Path(model_.parent) / "results" / f"{model_.stem}.building_res.mat"
        )
        if not result_path.exists():
            print(
                f"{CROSS_MARK} Simulation failed. Result file not found in {result_path}."
            )
            return
        progress.remove_task(task)
        print(f"{CHECKMARK} Simulation results available at {result_path}")

        task = progress.add_task(
            description="Creating report ...",
            total=None,
        )
        reporting = ModelDocumentation.from_network(
            network,
            result=ResultFile(path=result_path),
        )
        html = to_html_reporting(reporting)
        report_path = Path(model_.parent / f"{model_.stem}.html")
        report_path.write_text(html)
        webbrowser.open(f"file://{report_path}")
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
        print(
            f"{CHECKMARK} Model generated successfully. Your system is compatible for model generation."
        )
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
        print(
            f"{CHECKMARK} Model simulated successfully. Your system is compatible for simulation."
        )


def report(model: Path | str, options: SimulationLibraryOptions) -> None:
    model = Path(model).resolve()
    network = convert_network(
        model.stem, model, library=Library.from_configuration(options.library_name)
    )
    reporting = ModelDocumentation.from_network(
        network,
        result=ResultFile(
            path=Path(model.parent) / "results" / f"{model.stem}.building_res.mat"
        ),
    )
    html = to_html_reporting(reporting)
    report_path = Path(model.parent / f"{model.stem}.html")
    report_path.write_text(html)
    webbrowser.open(f"file://{report_path}")


if __name__ == "__main__":
    app()
