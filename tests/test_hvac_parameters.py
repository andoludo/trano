"""The physical parameters of the HVAC elements reach the Modelica models of every library."""

import re
from pathlib import Path

import pytest

from trano import elements
from tests.fixtures.simple_space_1 import simple_space_1_fixture
from tests.golden import remove_trano_package
from trano.data_models.conversion import convert_network
from trano.elements import param_from_config
from trano.elements.base import BaseElement
from trano.elements.library.library import Library
from trano.topology import Network

MODELS = Path(__file__).parent.joinpath("models")
P = param_from_config


def rendered(element: BaseElement, library: str = "Buildings") -> str:
    """The declaration of a single element, and the Trano package of the network."""
    network = Network(name="hvac", library=Library.from_configuration(library))
    element.assign_library_property(network.library)
    element.position.set_global(0.0, 0.0)
    element.position.set_container(0.0, 0.0)
    component = element.model(network)
    assert component is not None
    return re.sub(r"\s+", " ", component.model)


def trano_package(library: str = "Buildings") -> str:
    network = Network(name="package", library=Library.from_configuration(library))
    network.add_boiler_plate_spaces([simple_space_1_fixture()])
    return re.sub(r"\s+", " ", network.model())


def yaml_model(tmp_path: Path, name: str, library: str, **lines_after: str) -> str:
    text = MODELS.joinpath(f"{name}.yaml").read_text()
    for anchor, line in lines_after.items():
        anchor_line = next(candidate for candidate in text.splitlines() if candidate.strip() == anchor)
        indent = " " * (len(anchor_line) - len(anchor_line.lstrip()))
        text = text.replace(anchor_line + "\n", anchor_line + "\n" + indent + line + "\n", 1)
    model = tmp_path.joinpath(f"{name}.yaml")
    model.write_text(text)
    return re.sub(r"\s+", " ", convert_network(name, model, library=Library.from_configuration(library)).model())


# --------------------------------------------------------------------------- #
# Valves
# --------------------------------------------------------------------------- #


def test_the_flow_coefficient_type_names_the_enumeration_of_the_library() -> None:
    parameters = P("Valve")(flow_coefficient_type="Kv", kv=5)

    assert "Kv=5.0" in rendered(elements.Valve(name="valve", parameters=parameters))
    assert "CvData=Buildings.Fluid.Types.CvTypes.Kv" in rendered(elements.Valve(name="valve", parameters=parameters))
    assert "CvData=IDEAS.Fluid.Types.CvTypes.Kv" in rendered(
        elements.Valve(name="valve", parameters=parameters), "IDEAS"
    )
    assert "CvData" not in rendered(elements.Valve(name="valve", parameters=P("Valve")()))

    three_way = elements.ThreeWayValve(name="valve", parameters=P("ThreeWayValve")(flow_coefficient_type="Cv", Cv=3))
    assert "Cv=3.0" in rendered(three_way) and "CvData=Buildings.Fluid.Types.CvTypes.Cv" in rendered(three_way)


def test_the_actuator_stroke_time_has_a_different_name_in_ideas() -> None:
    valve = elements.Valve(name="valve", parameters=P("Valve")(actuator_stroke_time=60))
    assert "strokeTime=60.0" in rendered(valve) and "riseTime" not in rendered(valve)
    assert "riseTime=60.0" in rendered(valve, "IDEAS") and "strokeTime" not in rendered(valve, "IDEAS")

    three_way = elements.ThreeWayValve(name="valve", parameters=P("ThreeWayValve")(actuator_stroke_time=90))
    assert "strokeTime=90.0" in rendered(three_way) and "riseTime=90.0" in rendered(three_way, "IDEAS")


def test_the_short_names_of_the_valve_parameters() -> None:
    parameters = P("Valve")(linearized="false", from_dp="false", standard_density=1000, equal_percentage_deviation=0.02)

    model = rendered(elements.Valve(name="valve", parameters=parameters))
    assert (
        "linearized=false" in model and "from_dp=false" in model and "rhoStd=1000.0" in model and "delta0=0.02" in model
    )


# --------------------------------------------------------------------------- #
# Boiler and heat pumps
# --------------------------------------------------------------------------- #


def test_the_boiler_takes_its_losses_mass_fuel_and_initial_temperature() -> None:
    parameters = P("Boiler")(
        loss_conductance=10,
        water_volume=0.05,
        dry_mass=40,
        insulation_conductivity=0.03,
        ambient_temperature=290,
        fuel="HeatingOilLowerHeatingValue",
        temperature_initial=300,
        tank_height=1.5,
        nominal_efficiency_temperature=340,
    )

    model = rendered(elements.Boiler(name="boiler", parameters=parameters))
    for modifier in (
        "UA=10.0",
        "VWat=0.05",
        "mDry=40.0",
        "kIns=0.03",
        "TAmbient=290.0",
        "T_start=300.0",
        "hTan=1.5",
        "T_nominal=340.0",
    ):
        assert modifier in model, modifier
    assert "fue = Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue()" in model
    assert "NaturalGasLowerHeatingValue" in rendered(elements.Boiler(name="boiler", parameters=P("Boiler")()))


def test_the_wrappers_forward_the_boiler_parameters_to_their_components() -> None:
    package = trano_package()

    for forwarded in (
        "UA=UA,",
        "VWat=VWat,",
        "mDry=mDry,",
        "T_nominal=T_nominal,",
        "deltaM=deltaM,",
        "kIns=kIns,",
        "T_start=T_start",
    ):
        assert forwarded in package, forwarded
    assert "FixedTemperature TAmb(T=TAmbient)" in package
    assert "m_flow=mSou_flow_nominal," in package and "TEvaHea_nominal=TEvaHea_nominal," in package
    assert 'T_start=293.15) "Boiler"' not in package


def test_the_supply_set_point_and_the_source_belong_to_their_variants() -> None:
    without_storage = elements.Boiler(
        name="boiler", variant="without_storage", parameters=P("Boiler")(supply_temperature_setpoint=340)
    )
    assert "TempSet=340.0" in rendered(without_storage)
    assert "NaturalGasHigherHeatingValue" in rendered(without_storage)
    assert "TempSet" not in rendered(
        elements.Boiler(name="boiler", parameters=P("Boiler")(supply_temperature_setpoint=340))
    )

    air_water = elements.Boiler(
        name="hp",
        variant="air_water_heat_pump",
        parameters=P("Boiler")(heat_pump_nominal_source_temperature=280, heat_pump_source_mass_flow=2),
    )
    assert "TEvaHea_nominal=280.0" in rendered(air_water) and "mSou_flow_nominal=2.0" in rendered(air_water)


def test_the_boiler_flow_resistance_is_only_linearized_when_asked() -> None:
    assert "linearizeFlowResistance=false" in rendered(elements.Boiler(name="boiler", parameters=P("Boiler")()))
    assert "linearizeFlowResistance=true" in rendered(
        elements.Boiler(name="boiler", parameters=P("Boiler")(linearized="true"))
    )


# --------------------------------------------------------------------------- #
# Air handling unit, VAV box, duct
# --------------------------------------------------------------------------- #


def test_the_default_air_handling_unit_takes_its_flow_only_when_given(tmp_path: Path) -> None:
    name = "single_zone_air_handling_unit_complex_vav"
    default = yaml_model(tmp_path, name, "Buildings")
    assert "mAir_flow_nominal=" not in remove_trano_package(default) and "dpBuiStaSet=" not in remove_trano_package(
        default
    )

    parameters = "parameters:\n        m_flow_nominal: 0.5\n        building_static_pressure: 20"
    given = yaml_model(
        tmp_path, name, "Buildings", **{"id: AHU:001": parameters + "\n        outdoor_air_per_area: 0.0005"}
    )
    assert "mAir_flow_nominal=0.5," in given and "dpBuiStaSet=20.0," in given and "ratOAFlo_A=0.0005," in given


def test_the_simple_air_handling_unit_takes_separate_fan_pressure_rises() -> None:
    ahu = elements.AirHandlingUnit(
        name="ahu", variant="test", parameters=P("AirHandlingUnit")(supply_dp_nominal=300, return_dp_nominal=150)
    )
    model = rendered(ahu)
    assert "dpSup_nominal=300.0" in model and "dpRet_nominal=150.0" in model
    assert "dp_nominal=dpSup_nominal," in trano_package() and "dp_nominal=dpRet_nominal," in trano_package()


def test_the_vav_box_and_the_duct_take_their_sizing(tmp_path: Path) -> None:
    name = "single_zone_air_handling_unit_complex_vav"
    default = yaml_model(tmp_path, name, "Buildings")
    assert "THeaWatInl_nominal=363.15," in default and "VRoo=100.0," in default
    assert "m_flow_nominal=100*1.2/3600, dp_nominal=40.0," in default

    given = yaml_model(
        tmp_path,
        name,
        "Buildings",
        **{
            "variant: complex": "parameters:\n            heating_water_inlet_temperature: 350\n            room_volume: 60",  # noqa: E501
            "id: DUCT:001": "parameters:\n            dp_nominal: 60\n            nominal_mass_flow_rate: 0.2",
        },
    )
    assert "THeaWatInl_nominal=350.0," in given and "VRoo=60.0," in given
    assert "m_flow_nominal=0.2, dp_nominal=60.0," in given


def test_the_pressure_independent_vav_damper_takes_its_pressure_drops() -> None:
    vav = elements.VAV(name="vav", parameters=P("VAV")(damper_dp_nominal=30, fixed_dp_nominal=70))
    model = rendered(vav)
    assert (
        "m_flow_nominal=100*1.2/3600" in model and "dpDamper_nominal=30.0" in model and "dpFixed_nominal=70.0" in model
    )
    assert "THeaWatInl_nominal" not in model


# --------------------------------------------------------------------------- #
# Photovoltaic and sensors
# --------------------------------------------------------------------------- #


def test_the_photovoltaic_array_takes_its_geometry_in_degrees() -> None:
    pv = elements.Photovoltaic(
        name="pv", parameters=P("Photovoltaic")(area=30, tilt=30, azimuth=90, inverter_efficiency=0.95)
    )
    model = rendered(pv)
    assert "A=30.0" in model and "eta=0.18" in model and "til=0.523599" in model and "azi=1.570796" in model
    assert "eta_DCAC=0.95" in model and "tilt=" not in model


def test_the_sensors_take_their_time_constant() -> None:
    sensor = elements.TemperatureSensor(name="sensor", parameters=P("TemperatureSensor")(time_constant=0))
    assert "tau=0.0" in rendered(sensor)
    meter = elements.HeatMeterSensor(name="meter", parameters=P("HeatMeterSensor")(time_constant=5))
    assert "tau=5.0" in rendered(meter)


def test_a_short_name_given_twice_with_different_values_is_an_error() -> None:
    with pytest.raises(ValueError, match="same parameter"):
        P("Radiator")(nominal_heating_power=1000, nominal_heating_power_positive_for_heating=2000)
    assert P("Radiator")(nominal_heating_power=1000).nominal_heating_power_positive_for_heating == 1000
