"""The figures of the results of an element: its figures (``trano.model_description``) plotted by trano-simulate."""

from collections.abc import Callable

from buildingspy.io.outputfile import Reader  # type: ignore[import-untyped]
from matplotlib.figure import Figure as pyFigure
from plotly.graph_objects import Figure as plotlyFigure  # type: ignore[import-untyped]
from trano_simulate.plot.figure import NamedFigure
from trano_simulate.plot.plot import plot, plot_figures, plot_plot_ly, plot_plot_ly_many

from trano.elements import BaseElement
from trano.model_description import element_figures

__all__ = ["plot", "plot_element", "plot_figures", "plot_plot_ly", "plot_plot_ly_many"]


def plot_element(
    data: Reader,
    element: BaseElement,
    plot_function: Callable[[Reader, NamedFigure], plotlyFigure | pyFigure] = plot,
) -> list[pyFigure]:
    return plot_figures(data, element_figures(element), plot_function)
