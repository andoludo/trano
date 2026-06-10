from pathlib import Path
from typing import TYPE_CHECKING, Any, Union

from buildingspy.io.outputfile import Reader  # type: ignore
from jinja2 import Environment, FileSystemLoader

from trano.plot.plot import plot_element, plot_plot_ly

if TYPE_CHECKING:
    from trano.elements import (
        BaseElement,
        BaseSimpleWall,
        BaseWall,
        Space,
        System,
    )
    from trano.reporting.reporting import ModelDocumentation

# autoescape is intentionally disabled: these templates render trusted internal
# data and the historical output was produced without escaping.
_TEMPLATES = Environment(
    loader=FileSystemLoader(str(Path(__file__).parent.joinpath("templates"))),
    autoescape=False,  # noqa: S701
)


def to_html_space(data: dict[str, Any]) -> str:
    return _TEMPLATES.get_template("space.html").render(data=data)


def to_html_system(data: dict[str, Any]) -> str:
    return _TEMPLATES.get_template("system.html").render(data=data)


def to_html_construction(data: dict[str, Any]) -> str:
    return _TEMPLATES.get_template("construction.html").render(data=data)


def get_figures(element_name: str, documentation: "ModelDocumentation") -> list:  # type: ignore
    # TODO: this feels like a duplicate with docx!!!! check why
    if documentation.result is None:
        return []
    mat = Reader(documentation.result.path, documentation.result.type)
    elements = [node for node in documentation.elements if node.name == element_name]
    if elements:
        element = elements[0]
        return plot_element(mat, element, plot_plot_ly)


def get_description() -> dict[str, Any]:
    from trano.elements import BaseParameter

    return {
        field.alias: (field.description or field.title)
        for cls in BaseParameter.__subclasses__()
        for field in cls.model_fields.values()
        if field.alias
    }


def _get_elements(
    elements: list["BaseElement"],
    element_type: type[Union["Space", "System", "BaseSimpleWall", "BaseWall"]],
) -> list["BaseElement"]:
    return [element for element in elements if isinstance(element, element_type)]


def _dump_list_attributes(element: "BaseElement", attribute_name: str, include_mapping: object) -> list[dict[int, Any]]:
    datas = []
    for el in getattr(element, attribute_name):
        data = el.model_dump(
            by_alias=True,
            exclude_none=True,
            mode="json",
            include=include_mapping,
        )
        if el.parameters:
            data["parameters"] = el.parameters.model_dump(by_alias=True, exclude_none=True)
        datas.append(data)
    return datas
