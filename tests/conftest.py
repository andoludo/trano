"""Root conftest: fixtures live in tests/fixtures and are re-exported here.

`clean_model`/`read_golden` and the golden-file comparison helpers live in
tests/golden.py.
"""

from tests.fixtures.libraries import buildings_library  # noqa: F401
from tests.fixtures.networks import (  # noqa: F401
    building_multiple_internal_walls,
    building_multiple_internal_walls_ideas,
    buildings_free_float_single_zone,
    buildings_free_float_single_zone_ahu_complex,
    buildings_free_float_three_zones,
    buildings_free_float_three_zones_spaces,
    buildings_free_float_two_zones,
    buildings_simple_hydronic,
    buildings_simple_hydronic_three_zones,
    buildings_two_rooms_with_storage,
    house_model,
    ideas_free_float_single_zone,
    ideas_free_float_three_zones,
    ideas_free_float_three_zones_spaces,
    ideas_many_spaces_simple_ventilation,
    ideas_simple_hydronic_no_occupancy,
    ideas_simple_hydronic_three_zones,
    many_spaces_simple_ventilation,
    one_spaces_air_handling_unit,
    space_1_different_construction_types_network,
    space_1_ideal_heating_network,
    space_with_door,
    two_spaces_air_handling_unit,
    vav_ventilation_control,
)
from tests.fixtures.paths import (  # noqa: F401
    parameters_path,
    result_data_container_path,
    result_data_path,
    schema,
    schema_original,
)
from tests.fixtures.spaces import (  # noqa: F401
    simple_space_1,
    simple_space_1_with_occupancy,
    space_1,
    space_1_different_construction_types,
    space_1_ideal_heating,
    space_1_no_occupancy,
    space_1_simple_ventilation,
    space_1_simple_ventilation_vav_control,
    space_2,
    space_2_simple_ventilation,
    space_3,
    space_with_same_properties,
)
