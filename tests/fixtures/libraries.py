import pytest

from trano.elements.library.library import Library

IDEAS_CO2_MEDIUM_CONSTANTS = """
replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);"""

BUILDINGS_MEDIUM_CONSTANTS = """package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""


def buildings_library_fixture() -> Library:
    buildings = Library.from_configuration("Buildings")
    buildings.constants = BUILDINGS_MEDIUM_CONSTANTS
    return buildings


def ideas_library_fixture(co2_medium: bool = True) -> Library:
    ideas = Library.from_configuration("IDEAS")
    if co2_medium:
        ideas.constants = IDEAS_CO2_MEDIUM_CONSTANTS
    return ideas


@pytest.fixture
def buildings_library() -> Library:
    return buildings_library_fixture()
