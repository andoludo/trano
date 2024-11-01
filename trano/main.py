import webbrowser
from pathlib import Path

from trano.data_models.conversion import convert_network
from trano.library.library import Library
from trano.reporting.html import to_html_reporting
from trano.reporting.reporting import ModelDocumentation
from trano.reporting.types import ResultFile
from trano.simulate.simulate import SimulationLibraryOptions, simulate
from trano.topology import Network
from trano.utils.utils import is_success


def _create_network(model: Path | str, library: str) -> Network:
    library_ = Library.from_configuration(library)
    model = Path(model).resolve()
    return convert_network(model.stem, model, library=library_)


def create_model(model: Path | str, library: str = "Buildings") -> None:
    network = _create_network(model, library)
    modelica_model = network.model()
    Path(model).resolve().with_suffix(".mo").write_text(modelica_model)


def simulate_model(model: Path | str, options: SimulationLibraryOptions) -> None:

    network = _create_network(model, options.library_name)
    model = Path(model).resolve()
    results = simulate(
        model.parent,
        network,
        options=options,
    )
    if not is_success(results):
        raise ValueError("Simulation failed")

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
