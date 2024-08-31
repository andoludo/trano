from tests.constructions.constructions import Constructions, Glasses
from trano.elements.types import Tilt, Azimuth
from trano.elements.control import VAVControl
from trano.elements import ExternalWall, FloorOnGround, Window
from trano.elements.space import Space
from trano.elements.system import VAV, DamperVariant, Duct, Occupancy


def space_1_simple_ventilation_fixture() -> Space:
    space_1 = Space(
        name="space_1",
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        ventilation_inlets=[
            Duct(name="pressure_drop_duct_in"),
            VAV(
                name="vav_in",
                control=VAVControl(name="vav_in_control"),
                variant="complex",
            ),
        ],
        ventilation_outlets=[Duct(name="pressure_drop_duct_out")],
    )

    return space_1


def space_2_simple_ventilation_fixture() -> Space:
    space_2 = Space(
        name="space_2",
        occupancy=Occupancy(name="occupancy_1"),
        external_boundaries=[
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_3", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_2",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        ventilation_inlets=[
            Duct(name="pressure_drop_duct_in_2"),
            VAV(
                name="vav_in_2",
                control=VAVControl(name="vav_in_control_2"),
                variant="complex",
            ),
        ],
        ventilation_outlets=[Duct(name="pressure_drop_duct_out_2")],
    )

    return space_2


def space_1_simple_ventilation_vav_control_fixture() -> Space:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        ventilation_inlets=[
            Duct(name="pressure_drop_duct_in"),
            VAV(
                name="vav_in",
                control=VAVControl(name="vav_in_control"),
                variant=DamperVariant.complex,
            ),
        ],
        ventilation_outlets=[],
    )

    return space_1
