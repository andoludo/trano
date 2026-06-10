import pytest

from tests.fixtures.simple_space_1 import simple_space_1_fixture
from tests.fixtures.spaces_with_different_construction_types import (
    space_1_different_construction_types_fixture,
    space_with_same_properties_fixture,
)
from tests.fixtures.spaces_with_emissions import (
    space_1_fixture,
    space_1_ideal_heating_fixture,
    space_1_no_occupancy_fixture,
    space_2_fixture,
    space_3_fixture,
)
from tests.fixtures.spaces_with_ventilation import (
    space_1_simple_ventilation_fixture,
    space_1_simple_ventilation_vav_control_fixture,
    space_2_simple_ventilation_fixture,
)
from trano.elements.space import Space


@pytest.fixture
def simple_space_1() -> Space:
    return simple_space_1_fixture()


@pytest.fixture
def simple_space_1_with_occupancy() -> Space:
    return simple_space_1_fixture(occupancy=True)


@pytest.fixture
def space_1() -> Space:
    return space_1_fixture()


@pytest.fixture
def space_2() -> Space:
    return space_2_fixture()


@pytest.fixture
def space_3() -> Space:
    return space_3_fixture()


@pytest.fixture
def space_1_no_occupancy() -> Space:
    return space_1_no_occupancy_fixture()


@pytest.fixture
def space_1_ideal_heating() -> Space:
    return space_1_ideal_heating_fixture()


@pytest.fixture
def space_1_different_construction_types() -> Space:
    return space_1_different_construction_types_fixture()


@pytest.fixture
def space_1_simple_ventilation() -> Space:
    return space_1_simple_ventilation_fixture()


@pytest.fixture
def space_2_simple_ventilation() -> Space:
    return space_2_simple_ventilation_fixture()


@pytest.fixture
def space_1_simple_ventilation_vav_control() -> Space:
    return space_1_simple_ventilation_vav_control_fixture()


@pytest.fixture
def space_with_same_properties() -> Space:
    return space_with_same_properties_fixture()
