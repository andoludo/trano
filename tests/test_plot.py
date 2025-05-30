from pathlib import Path

import pytest
from buildingspy.io.outputfile import Reader

from trano.elements.space import Space
from trano.plot.plot import plot, plot_element, plot_plot_ly, plot_plot_ly_many
from trano.topology import Network


@pytest.fixture
def data_reader(result_data_path: Path) -> Reader:
    return Reader(
        result_data_path,
        "openmodelica",
    )


@pytest.fixture
def data_reader_container(result_data_container_path: Path) -> Reader:
    return Reader(
        result_data_container_path,
        "openmodelica",
    )


def test_plot_one_figure(
    buildings_two_rooms_with_storage: Network, data_reader: Reader
) -> None:
    space = [
        s for s in buildings_two_rooms_with_storage.graph.nodes if isinstance(s, Space)
    ]
    figure = plot(data_reader, space[0].figures[0])
    figure_plotly = plot_plot_ly(data_reader, space[0].figures[0])
    assert figure
    assert figure_plotly


def test_plot_one_figure_container(
    building_multiple_internal_walls: Network, data_reader_container: Reader
) -> None:
    building_multiple_internal_walls.model()
    space = [
        s for s in building_multiple_internal_walls.graph.nodes if isinstance(s, Space)
    ]
    figure = plot(data_reader_container, space[0].figures[0])
    figure_plotly = plot_plot_ly(data_reader_container, space[0].figures[0])
    assert figure
    assert figure_plotly


def test_plot_element(
    buildings_two_rooms_with_storage: Network, data_reader: Reader
) -> None:
    space = [
        s for s in buildings_two_rooms_with_storage.graph.nodes if isinstance(s, Space)
    ]
    figures = plot_element(data_reader, space[0])
    figures_plotly = plot_element(data_reader, space[0], plot_function=plot_plot_ly)
    assert len(figures) == 6
    assert len(figures_plotly) == 6


def test_plot_plot_ly_many(
    buildings_two_rooms_with_storage: Network, data_reader: Reader
) -> None:
    space = [
        s for s in buildings_two_rooms_with_storage.graph.nodes if isinstance(s, Space)
    ]
    for figures in list(zip(*[s.figures for s in space])):
        figures_plotly = plot_plot_ly_many(data_reader, figures, show=True)
        assert figures_plotly
