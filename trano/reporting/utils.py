from typing import TYPE_CHECKING, Any, Dict, List, Type, Union

from buildingspy.io.outputfile import Reader  # type: ignore
from jinja2 import Template

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


def to_html_space(data: Dict[str, Any]) -> str:
    template = Template(
        """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>External Boundaries Table</title>
        <style>
            body {
                font-family: Arial, sans-serif;
            }
            .fancy-table {
                width: 100%;
                border-collapse: collapse;
                margin: 20px 0;
                font-size: 1em;
                text-align: left;
            }
            .fancy-table th, .fancy-table td {
                padding: 12px;
                border: 1px solid #ddd;
            }
            .fancy-table th {
                background-color: #f4f4f4;
                font-weight: bold;
            }
            .fancy-table tr:nth-child(even) {
                background-color: #f9f9f9;
            }
            .fancy-table caption {
                font-size: 1.5em;
                margin-bottom: 10px;
                font-weight: bold;
            }
        </style>
    </head>
    <body>
    {% for parameter_ in [data.parameters,data.occupancy] %}
    <table>
        <thead>
            <tr>
                {% for key in parameter_.keys() %}
                    <th><strong>{{ key }}</strong></th>
                {% endfor %}
            </tr>
        </thead>
        <tbody>
            <tr>
                {% for value in parameter_.values() %}
                    <td>{{ value }}</td>
                {% endfor %}
            </tr>
        </tbody>
    </table>
    {% endfor %}
    <table class="fancy-table">
        <thead>
            <tr>
                <th>Name</th>
                <th>Azimuth</th>
                <th>Construction Name</th>
                <th>Surface</th>
                <th>Tilt</th>
            </tr>
        </thead>
        <tbody>
        {% for boundary in data.external_boundaries + data.internal_elements %}
            <tr>
                <td>{{ boundary.name }}</td>
                <td>{{ boundary.azimuth }}</td>
                <td>{{ boundary.construction }}</td>
                <td>{{ boundary.surface }}</td>
                <td>{{ boundary.tilt }}</td>
            </tr>
        {% endfor %}
        </tbody>

        {% for emission in data.emissions %}
    <table>
        <thead>
            <tr>
                {% for key in emission.keys() %}
                    <th><strong>{{ key }}</strong></th>
                {% endfor %}
            </tr>
        </thead>
        <tbody>
            <tr>
                {% for value in emission.values() %}
                    <td>{{ value }}</td>
                {% endfor %}
            </tr>
        </tbody>
    </table>
    {% endfor %}

    </body>
    </html>
    """
    )

    return template.render(data=data)


def to_html_system(data: Dict[str, Any]) -> str:
    template = Template(
        """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>External Boundaries Table</title>
        <style>
            body {
                font-family: Arial, sans-serif;
            }
            .fancy-table {
                width: 100%;
                border-collapse: collapse;
                margin: 20px 0;
                font-size: 1em;
                text-align: left;
            }
            .fancy-table th, .fancy-table td {
                padding: 12px;
                border: 1px solid #ddd;
            }
            .fancy-table th {
                background-color: #f4f4f4;
                font-weight: bold;
            }
            .fancy-table tr:nth-child(even) {
                background-color: #f9f9f9;
            }
            .fancy-table caption {
                font-size: 1.5em;
                margin-bottom: 10px;
                font-weight: bold;
            }
        </style>
    </head>
    <body>

    <table>
        <thead>
            <tr>
                {% for key in data.keys() %}
                    <th><strong>{{ key }}</strong></th>
                {% endfor %}
            </tr>
        </thead>
        <tbody>
            <tr>
                {% for value in data.values() %}
                    <td>{{ value }}</td>
                {% endfor %}
            </tr>
        </tbody>
    </table>


    </body>
    </html>
    """
    )

    return template.render(data=data)


def to_html_construction(data: Dict[str, Any]) -> str:
    template = Template(
        """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Layer Information Table</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                margin: 20px;
            }
            table {
                width: 100%;
                border-collapse: collapse;
                margin-bottom: 20px;
                font-size: 1em;
            }
            th, td {
                padding: 12px;
                border: 1px solid #ddd;
                text-align: left;
            }
            th {
                background-color: #f4f4f4;
                font-weight: bold;
            }
            tr:nth-child(even) {
                background-color: #f9f9f9;
            }
            caption {
                font-size: 0.75em;
                margin-bottom: 10px;
                font-weight: bold;
                text-align: left;
            }
        </style>
    </head>
    <body>

    <table>
        <caption>Layers for {{ data.name }}</caption>
        <thead>
            <tr>
                <th>Name</th>
                <th>c</th>
                <th>epsLw</th>
                <th>epsSw</th>
                <th>k</th>
                <th>rho</th>
                <th>Thickness</th>
            </tr>
        </thead>
        <tbody>
        {% for layer in data.layers %}
            <tr>
                <td>{{ layer.name }}</td>
                <td>{{ layer.c }}</td>
                <td>{{ layer.epsLw }}</td>
                <td>{{ layer.epsSw }}</td>
                <td>{{ layer.k }}</td>
                <td>{{ layer.rho }}</td>
                <td>{{ layer.thickness }}</td>
            </tr>
        {% endfor %}
        </tbody>
    </table>

    </body>
    </html>
    """
    )

    # Render the template with the data
    return template.render(data=data)


def get_figures(element_name: str, documentation: "ModelDocumentation") -> list:  # type: ignore
    # TODO: this feels like a duplicate with docx!!!! check why
    if documentation.result is None:
        return []
    mat = Reader(documentation.result.path, documentation.result.type)
    elements = [node for node in documentation.elements if node.name == element_name]
    if elements:
        element = elements[0]
        return plot_element(mat, element, plot_plot_ly)


def get_description() -> Dict[str, Any]:
    from trano.elements import BaseParameter

    return {
        field.alias: (field.description or field.title)
        for cls in BaseParameter.__subclasses__()
        for field in cls.model_fields.values()
        if field.alias
    }


def _get_elements(
    elements: List["BaseElement"],
    element_type: Type[Union["Space", "System", "BaseSimpleWall", "BaseWall"]],
) -> List["BaseElement"]:
    return [element for element in elements if isinstance(element, element_type)]


def _dump_list_attributes(
    element: "BaseElement", attribute_name: str, include_mapping: object
) -> List[Dict[int, Any]]:
    datas = []
    for el in getattr(element, attribute_name):
        data = el.model_dump(
            by_alias=True,
            exclude_none=True,
            mode="json",
            include=include_mapping,
        )
        if el.parameters:
            data["parameters"] = el.parameters.model_dump(
                by_alias=True, exclude_none=True
            )
        datas.append(data)
    return datas
