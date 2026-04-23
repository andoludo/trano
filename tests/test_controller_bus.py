from trano.elements.data_bus.controller_bus import ControllerBus
from trano.elements.library.parameters import PARAMETERS
from trano.elements.system import Occupancy

CONTROLLER_BUS_DATA = {
    "template": "Trano.Controls.BaseClasses.DataBus dataBus\n    "
    "annotation (Placement(transformation(\n  "
    "extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));",
    "real_inputs": [
        {"name": "DataSource", "component": "co2ToOccupancy2", "port": "co2", "target": {"main": "DataSource"}},
        {"name": "DataSource", "component": "co2ToOccupancy3", "port": "co2", "target": {"main": "DataSource"}},
    ],
    "real_outputs": [],
    "integer_inputs": [],
    "integer_outputs": [
        {
            "name": "Occupied",
            "component": "occSch2",
            "port": "occupied",
            "multi": False,
            "target": {"main": "element.space_name", "sub": None, "evaluated_element": "", "value": ""},
            "input_template": "Modelica.Blocks.Sources.IntegerExpression",
            "default": 0,
            "power": None,
            "input_model": "",
            "input_name": "Occupied",
        }
    ],
    "boolean_inputs": [],
    "boolean_outputs": [],
}


def test_controller_bus_parameter_data_normal() -> None:
    controller_bus = ControllerBus.model_validate(CONTROLLER_BUS_DATA)
    occupancy_parameters = PARAMETERS["Occupancy"]
    occupancy = Occupancy(parameters=occupancy_parameters(), space_name="space_1")
    ports = controller_bus.bus_ports(occupancy)
    assert set(ports) == {"connect(dataBus.OccupiedSpace_1, occSch2.occupied);"}


def test_controller_bus_parameter_data_with_data() -> None:
    controller_bus = ControllerBus.model_validate(CONTROLLER_BUS_DATA)
    occupancy_parameters = PARAMETERS["Occupancy"]
    occupancy = Occupancy(
        parameters=occupancy_parameters.model_validate(
            {
                "data": [
                    {"component": "co2ToOccupancy2", "variable": "data_1"},
                    {"component": "co2ToOccupancy3", "variable": "data_2"},
                    {"component": "co2ToOccupancy4", "variable": "data_5"},
                ]
            }
        ),
        space_name="space_1",
    )
    ports = controller_bus.bus_ports(occupancy)
    assert set(ports) == {
        "connect(dataBus.OccupiedSpace_1, occSch2.occupied);",
        "connect(dataBus.data_1, co2ToOccupancy2.co2);",
        "connect(dataBus.data_2, co2ToOccupancy3.co2);",
    }
