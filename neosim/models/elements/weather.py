from typing import Callable, List

from pydantic import Field

from neosim.models.elements.base import AvailableLibraries, LibraryData, Port
from neosim.models.elements.boundary import Boundary
from neosim.models.elements.space import Space
from neosim.models.elements.system import BaseWeather


class WeatherComponent(LibraryData):
    template: str = """    Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                {{ element.name }}(filNam =
        Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
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


class Weather(BaseWeather):
    libraries_data: AvailableLibraries = AvailableLibraries(
        buildings=[WeatherComponent]
    )
