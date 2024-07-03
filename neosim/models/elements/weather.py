from typing import Callable, List, Optional

from pydantic import Field

from neosim.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    LibraryData,
    Port,
)
from neosim.models.elements.boundary import Boundary
from neosim.models.elements.space import Space
from neosim.models.elements.system import BaseWeather


class WeatherPath:
    uccle: str = (
        "Modelica.Utilities.Files.loadResource"
        '("modelica://IDEAS/Resources/weatherdata'
        '/BEL_VLG_Uccle.064470_TMYx.2007-2021.mos")'
    )
    vliet_2021: str = (
        "Modelica.Utilities.Files.loadResource"
        '("modelica://IDEAS 3.0.0/Resources/'
        'weatherdata/Vliet_2021.mos")'
    )
    chicago: str = (
        "Modelica.Utilities.Files.loadResource"
        '("modelica://Buildings/Resources/weatherdata/'
        'USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")'
    )
    denver: str = (
        "Modelica.Utilities.Files.loadResource("
        '"modelica://Buildings/Resources/weatherdata/'
        'USA_CO_Denver.Intl.AP.725650_TMY3.mos")'
    )
    san_francisco: str = (
        "Modelica.Utilities.Files.loadResource"
        '("modelica://Buildings/Resources/weatherdata/'
        'USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")'
    )


class WeatherParameters(BaseParameter):
    path: Optional[str] = Field(None, alias="filNam", title="Path to the weather file")


class BuildingsWeatherComponent(LibraryData):
    template: str = """
    {% if macros.render_parameters(parameters) | safe %}
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
    {% else %}
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                {{ element.name }}(filNam=Modelica.Utilities.Files.loadResource
        ("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    {% endif %}
    """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space, Boundary],
                names=["weaBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class IdeasWeatherComponent(LibraryData):
    template: str = """
    {% if macros.render_parameters(parameters) | safe %}
    inner IDEAS.BoundaryConditions.SimInfoManager
    sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort, {{ macros.render_parameters(parameters) | safe}}) "Data reader"
    {% raw %}annotation (Placement(transformation(extent={{-96,76},{-76,96}})));{% endraw %}
    {% else %}
        inner IDEAS.BoundaryConditions.SimInfoManager
    sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort) "Data reader"
    {% raw %}annotation (Placement(transformation(extent={{-96,76},{-76,96}})));{% endraw %}
    {% endif %}
    """
    ports_factory: Callable[[], List[Port]] = Field(default=list)


class Weather(BaseWeather):
    parameters: WeatherParameters = Field(default=WeatherParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        buildings=[BuildingsWeatherComponent], ideas=[IdeasWeatherComponent]
    )
