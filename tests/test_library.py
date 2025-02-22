from codecs import namereplace_errors
from typing import Dict, Any

import pytest

from tests.constructions.constructions import Constructions, Glasses
from trano.elements import (
    VAV,
    VAVControl,
    Space,
    ExternalWall,
    FloorOnGround,
    Window,
    DataBus,
)
from trano.elements.bus import get_non_connected_ports
from trano.elements.common_base import MediumTemplate
from trano.elements.library.base import LibraryData
from trano.elements.library.components import COMPONENTS

from trano.elements.library.library import Library, Templates
from trano.elements.system import Weather, Occupancy
from trano.elements.types import Azimuth, Tilt
from trano.topology import Network


@pytest.fixture
def vav_library_data():
    libraries_data = {
        "components": [
            {
                "classes": ["VAVControl"],
                "component_template": {
                    "bus": {
                        "boolean_inputs": [
                            {
                                "component": "rehBoxCon",
                                "name": "u1Win",
                                "port": "u1Win",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "u1Occ",
                                "port": "u1Occ",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "uHeaOff",
                                "port": "uHeaOff",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "u1Fan",
                                "port": "u1Fan",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "u1HotPla",
                                "port": "u1HotPla",
                                "target": {"main": "element.space_name"},
                            },
                        ],
                        "integer_inputs": [
                            {
                                "component": "rehBoxCon",
                                "default": 1,
                                "name": "uOpeMod",
                                "port": "uOpeMod",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "oveFloSet",
                                "port": "oveFloSet",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "oveDamPos",
                                "port": "oveDamPos",
                                "target": {"main": "element.space_name"},
                            },
                        ],
                        "integer_outputs": [
                            {
                                "component": "rehBoxCon",
                                "name": "yZonTemResReq",
                                "port": "yZonTemResReq",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yZonPreResReq",
                                "port": "yZonPreResReq",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yHeaValResReq",
                                "port": "yHeaValResReq",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yHotWatPlaReq",
                                "port": "yHotWatPlaReq",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yLowFloAla",
                                "port": "yLowFloAla",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yFloSenAla",
                                "port": "yFloSenAla",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yLeaDamAla",
                                "port": "yLeaDamAla",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yLeaValAla",
                                "port": "yLeaValAla",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yLowTemAla",
                                "port": "yLowTemAla",
                                "target": {"main": "element.controllable_element.name"},
                            },
                        ],
                        "real_inputs": [
                            {
                                "component": "rehBoxCon",
                                "name": "TZon",
                                "port": "TZon",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "default": 298.15,
                                "name": "TCooSet",
                                "port": "TCooSet",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "default": 293.15,
                                "name": "THeaSet",
                                "port": "THeaSet",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "ppmCO2Set",
                                "port": "ppmCO2Set",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "ppmCO2",
                                "port": "ppmCO2",
                                "target": {"main": "element.space_name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "TDis",
                                "port": "TDis",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "VDis_flow",
                                "port": "VDis_flow",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "default": 293.15,
                                "name": "TSupSet",
                                "port": "TSupSet",
                                "target": {"main": "element.space_name"},
                            },
                        ],
                        "real_outputs": [
                            {
                                "component": "rehBoxCon",
                                "name": "VSet_flow",
                                "port": "VSet_flow",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yDam",
                                "port": "yDam",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yVal",
                                "port": "yVal",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "VAdjPopBreZon_flow",
                                "port": "VAdjPopBreZon_flow",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "VAdjAreBreZon_flow",
                                "port": "VAdjAreBreZon_flow",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "VMinOA_flow",
                                "port": "VMinOA_flow",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "VZonAbsMin_flow",
                                "port": "VZonAbsMin_flow",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "VZonDesMin_flow",
                                "port": "VZonDesMin_flow",
                                "target": {"main": "element.controllable_element.name"},
                            },
                            {
                                "component": "rehBoxCon",
                                "name": "yCO2",
                                "port": "yCO2",
                                "target": {"main": "element.controllable_element.name"},
                            },
                        ],
                        "template": "Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(\n  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));",
                    },
                    "category": "control",
                    "template": "model VAVControl{{ element.name | capitalize}}\nBuildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller rehBoxCon(\nvenStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,\nhave_winSen=True,\nhave_occSen=True,\nhave_CO2Sen=True,\nhave_hotWatCoi=True,\nVOccMin_flow=0.003,\nVAreMin_flow=0.003,\nVAreBreZon_flow=0.003,\nVPopBreZon_flow=0.003,\nVMin_flow=0.003,\nVCooMax_flow=0.003,\nVHeaMin_flow=0.003,\nVHeaMax_flow=0.003)\n{% raw %}annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); {% endraw %}\n{{bus_template}}\nequation\n{{bus_ports | safe}}\nend VAVControl{{ element.name  | capitalize}};",
                },
                "figures": [],
                "library": "default",
                "parameter_processing": {"function": "default_parameters"},
                "ports": [
                    {
                        "multi_connection": True,
                        "names": ["dataBus"],
                        "targets": ["System", "DataBus"],
                        "use_counter": False,
                        "medium": "data",
                        "flow": "undirected",
                    }
                ],
                "template": "\n    {{package_name}}.Trano.Controls.ventilation.VAVControl{{ element.name | capitalize}}\n    {{ element.name }}",
                "variant": "default",
            },
            {
                "classes": ["VAV"],
                "component_template": {
                    "bus": {
                        "real_inputs": [
                            {
                                "component": "vav",
                                "name": "yDam",
                                "port": "y",
                                "target": {"main": "element.name"},
                            }
                        ],
                        "real_outputs": [
                            {
                                "component": "vav",
                                "name": "y_actual",
                                "port": "y_actual",
                                "target": {"main": "element.name"},
                            },
                            {
                                "component": "senVolFlo",
                                "name": "VDis_flow",
                                "port": "V_flow",
                                "target": {"main": "element.control.name"},
                            },
                            {
                                "component": "senTem",
                                "name": "TDis",
                                "port": "T",
                                "target": {"main": "element.control.name"},
                            },
                        ],
                    },
                    "category": "ventilation",
                    "template": "\nmodel VAVBox{{ element.name | capitalize}}\nextends {{ package_name }}.Trano.Fluid.Ventilation.PartialVAVBox;\n{{bus_template}}\nequation\n{{bus_ports | safe}}\n end VAVBox{{ element.name | capitalize}};\n ",
                },
                "figures": [],
                "ports": [],
                "library": "default",
                "parameter_processing": {"function": "default_parameters"},
                "template": "  {{ package_name }}.Trano.\n    Fluid.Ventilation.VAVBox{{ element.name | capitalize }}\n     {{ element.name }}(\n    redeclare package MediumA = Medium,\n    mCooAir_flow_nominal=100*1.2/3600,\n    mHeaAir_flow_nominal=100*1.2/3600,\n    VRoo=100,\n    allowFlowReversal=False,\n    THeaWatInl_nominal=90,\n    THeaWatOut_nominal=60,\n    THeaAirInl_nominal=30,\n    THeaAirDis_nominal=25\n    )",
                "variant": "complex",
            },
        ]
    }
    return libraries_data


def test_dynamic_template_vav(vav_library_data):
    vav_library = LibraryData.model_validate(vav_library_data["components"][1])
    vav_control_library = LibraryData.model_validate(vav_library_data["components"][0])
    vav = VAV(variant="complex", libraries_data=[vav_library])
    vav_control = VAVControl(libraries_data=[vav_control_library])
    vav_control.space_name = "space_vav"
    vav_control.controllable_element = vav
    vav.control = vav_control
    library = Library.load_default()
    vav.assign_library_property(library)
    vav_control.assign_library_property(library)
    rendered_template = vav.component_template.render(
        "test", vav, vav.processed_parameters(library)
    )
    vav_control_template = vav_control.component_template.render(
        "test", vav_control, vav_control.processed_parameters(library)
    )
    get_non_connected_ports([vav, vav_control])
    assert vav_control_template == (
        "model VAVControlVavcontrol_0\n"
        "Buildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller "
        "rehBoxCon(\n"
        "venStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,\n"
        "have_winSen=True,\n"
        "have_occSen=True,\n"
        "have_CO2Sen=True,\n"
        "have_hotWatCoi=True,\n"
        "VOccMin_flow=0.003,\n"
        "VAreMin_flow=0.003,\n"
        "VAreBreZon_flow=0.003,\n"
        "VPopBreZon_flow=0.003,\n"
        "VMin_flow=0.003,\n"
        "VCooMax_flow=0.003,\n"
        "VHeaMin_flow=0.003,\n"
        "VHeaMax_flow=0.003)\n"
        "annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); "
        "Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(\n"
        "  extent={{-120,-18},{-80,22}}), "
        "iconTransformation(extent={{-120,62},{-78,98}})));\n"
        "equation\n"
        "connect(dataBus.TZonSpace_vav, rehBoxCon.TZon);\n"
        "connect(dataBus.TCooSetSpace_vav, rehBoxCon.TCooSet);\n"
        "connect(dataBus.THeaSetSpace_vav, rehBoxCon.THeaSet);\n"
        "connect(dataBus.ppmCO2SetSpace_vav, rehBoxCon.ppmCO2Set);\n"
        "connect(dataBus.ppmCO2Space_vav, rehBoxCon.ppmCO2);\n"
        "connect(dataBus.TSupSetSpace_vav, rehBoxCon.TSupSet);\n"
        "connect(dataBus.uOpeModSpace_vav, rehBoxCon.uOpeMod);\n"
        "connect(dataBus.oveFloSetSpace_vav, rehBoxCon.oveFloSet);\n"
        "connect(dataBus.oveDamPosSpace_vav, rehBoxCon.oveDamPos);\n"
        "connect(dataBus.u1WinSpace_vav, rehBoxCon.u1Win);\n"
        "connect(dataBus.u1OccSpace_vav, rehBoxCon.u1Occ);\n"
        "connect(dataBus.uHeaOffSpace_vav, rehBoxCon.uHeaOff);\n"
        "connect(dataBus.u1FanSpace_vav, rehBoxCon.u1Fan);\n"
        "connect(dataBus.u1HotPlaSpace_vav, rehBoxCon.u1HotPla);\n"
        "connect(dataBus.TDisVavcontrol_0, rehBoxCon.TDis);\n"
        "connect(dataBus.VDis_flowVavcontrol_0, rehBoxCon.VDis_flow);\n"
        "connect(dataBus.VAdjPopBreZon_flowVavcontrol_0, "
        "rehBoxCon.VAdjPopBreZon_flow);\n"
        "connect(dataBus.VAdjAreBreZon_flowVavcontrol_0, "
        "rehBoxCon.VAdjAreBreZon_flow);\n"
        "connect(dataBus.VMinOA_flowVavcontrol_0, rehBoxCon.VMinOA_flow);\n"
        "connect(dataBus.yZonTemResReqVavcontrol_0, rehBoxCon.yZonTemResReq);\n"
        "connect(dataBus.yZonPreResReqVavcontrol_0, rehBoxCon.yZonPreResReq);\n"
        "connect(dataBus.yHeaValResReqVavcontrol_0, rehBoxCon.yHeaValResReq);\n"
        "connect(dataBus.VSet_flowVav_0, rehBoxCon.VSet_flow);\n"
        "connect(dataBus.yDamVav_0, rehBoxCon.yDam);\n"
        "connect(dataBus.yValVav_0, rehBoxCon.yVal);\n"
        "connect(dataBus.VZonAbsMin_flowVav_0, rehBoxCon.VZonAbsMin_flow);\n"
        "connect(dataBus.VZonDesMin_flowVav_0, rehBoxCon.VZonDesMin_flow);\n"
        "connect(dataBus.yCO2Vav_0, rehBoxCon.yCO2);\n"
        "connect(dataBus.yHotWatPlaReqVav_0, rehBoxCon.yHotWatPlaReq);\n"
        "connect(dataBus.yLowFloAlaVav_0, rehBoxCon.yLowFloAla);\n"
        "connect(dataBus.yFloSenAlaVav_0, rehBoxCon.yFloSenAla);\n"
        "connect(dataBus.yLeaDamAlaVav_0, rehBoxCon.yLeaDamAla);\n"
        "connect(dataBus.yLeaValAlaVav_0, rehBoxCon.yLeaValAla);\n"
        "connect(dataBus.yLowTemAlaVav_0, rehBoxCon.yLowTemAla);\n"
        "end VAVControlVavcontrol_0;"
    )
    assert rendered_template == (
        "model VAVBoxVav_0\n"
        "extends test.Trano.Fluid.Ventilation.PartialVAVBox;\n"
        "Trano.Controls.BaseClasses.DataBus dataBus\n"
        "    annotation (Placement(transformation(\n"
        "  extent={{-120,-18},{-80,22}}), "
        "iconTransformation(extent={{-120,62},{-78,98}})));\n"
        "equation\n"
        "connect(dataBus.yDamVav_0, vav.y);\n"
        "connect(dataBus.y_actualVav_0, vav.y_actual);\n"
        "connect(dataBus.VDis_flowVavcontrol_0, senVolFlo.V_flow);\n"
        "connect(dataBus.TDisVavcontrol_0, senTem.T);\n"
        " end VAVBoxVav_0;\n"
        " "
    )


@pytest.fixture
def weather(request):
    return Weather(
        name="weather_1", libraries_data=[LibraryData.model_validate(request.param)]
    )


@pytest.fixture
def simple_space_template(request):
    return Space(
        name="space_1",
        libraries_data=[LibraryData.model_validate(request.param)],
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
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
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
    )


def iso_13790():
    zone_space_template = """
Zone5R1C.Zone {{ element.name }}(
{{ macros.render_parameters(parameters) | safe }},
{%- for boundary in element.boundaries -%}
    {%- if boundary.type == 'ExternalWallVerticalOnly' -%}
        {%- if boundary.number %}
            AWal={{ macros.join_list(boundary.surfaces) }},
            nOrientations={{ boundary.number }},
            surAzi={{ macros.join_list(boundary.azimuths_to_radians()) }},
            surTil={{ macros.join_list(boundary.tilts_to_radians()) }},
            UWal = {{boundary.average_u_value}},
            {%- endif %}
        {%- endif %}
    {%- if boundary.type == 'BaseWindow' -%}
        {%- if boundary.number %}
            AWin={{ macros.join_list(boundary.window_area_by_orientation) }},
            UWin={{ boundary.average_u_value }},
            {%- endif %}
        {%- endif %}
    {%- if boundary.type == 'FloorOnGround' -%}
        {%- if boundary.number %}
            AFlo={{ boundary.surfaces[0] }},
            UFlo={{ boundary.average_u_value }},
            {%- endif %}
        {%- endif %}
    {%- if boundary.type == 'ExternalWallRoof' -%}
        {%- if boundary.number %}
            ARoo={{ boundary.surfaces[0] }},
            URoo={{ boundary.average_u_value }},
            {%- endif %}
        {%- endif %}
{%- endfor %}
{% raw %}
redeclare replaceable Buildings.ThermalZones.ISO13790.Data.Light buiMas,
gFac=0.5) "Thermal zone"
annotation (Placement(transformation(extent={{26,-12},{54,16}})));
{% endraw %}
    """
    return {
        "classes": ["Space"],
        "library": "reduced_order",
        "parameter_processing": {
            "function": "modify_alias",
            "parameter": {"floor_area": "AFlo", "volume": "VRoo"},
        },
        "ports": [
            {
                "multi_connection": True,
                "names": ["surf_surBou"],
                "targets": ["BaseInternalElement"],
                "medium": "heat",
                "flow": "interchangeable_port",
            },
            {
                "names": ["qGai_flow"],
                "targets": ["BaseOccupancy"],
                "medium": "data",
                "flow": "undirected",
            },
            {
                "names": ["weaBus"],
                "targets": ["BaseWeather"],
                "medium": "data",
                "flow": "undirected",
            },
            {
                "names": ["heaPorRad"],
                "targets": ["Emission"],
                "medium": "heat",
                "flow": "radiative",
            },
            {
                "names": ["heaPorAir"],
                "multi_connection": True,
                "use_counter": False,
                "targets": ["Emission"],
                "medium": "heat",
                "flow": "convective",
            },
            {
                "names": ["heaPorAir"],
                "targets": ["DataBus"],
                "medium": "heat",
                "flow": "convective",
            },
            {
                "names": ["heaPorAir"],
                "targets": ["VAVControl"],
                "medium": "heat",
                "flow": "convective",
            },
            {
                "flow": "inlet_or_outlet",
                "multi_connection": True,
                "names": ["ports"],
                "medium": "fluid",
                "targets": ["Ventilation", "Control", "DataBus"],
            },
        ],
        "template": zone_space_template,
        "variant": "default",
    }


@pytest.mark.parametrize("simple_space_template", [(iso_13790())], indirect=True)
def test_use_new_space_library(simple_space_template: Space):
    library = Library(
        name="reduced_order",
        merged_external_boundaries=False,
        templates=Templates(construction="", glazing="", main=""),
        medium=MediumTemplate(air="AixLib.Media.Air", water="AixLib.Media.Water"),
    )
    network = Network(name="house_model", library=library)
    network.add_boiler_plate_spaces([simple_space_template])
    network.connect()
    simple_space_template.get_neighhors(network.graph)
    model = simple_space_template.model(network)
    assert model


def reduced_order_weather():
    template = """
      AixLib.BoundaryConditions.WeatherData.ReaderTMY3
    {{ element.name }}
    {% raw %}annotation (Placement(transformation(extent={{-52,70},{-32,90}})));{% endraw %}
    """
    return {
        "classes": ["Weather"],
        "figures": [],
        "library": "reduced_order",
        "parameter_processing": {"function": "default_parameters"},
        "ports": [
            {
                "multi_connection": True,
                "use_counter": False,
                "names": ["weaBus"],
                "targets": ["Space"],
                "medium": "data",
                "flow": "undirected",
            },
        ],
        "template": template,
        "variant": "default",
    }


def reduced_order():
    zone_space_template = """
AixLib.ThermalZones.ReducedOrder.ThermalZone.ThermalZone {{ element.name }}(
    use_moisture_balance=True,
    ROM(extWallRC(thermCapExt(each der_T(fixed=True))), intWallRC(thermCapInt(
            each der_T(fixed=True)))),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    redeclare package Medium = Medium,
    internalGainsMode=1,
    nPorts=2,
    T_start=293.15,
    zoneParam=AixLib.DataBase.ThermalZones.ZoneBaseRecord(
    T_start=293.15,
    VAir=6700.0,
    AZone=1675.0,
    hRad=5,
    lat=0.87266462599716,
    nOrientations=5,
    AWin={108.5,19.0,108.5,19.0,0},
    ATransparent={108.5,19.0,108.5,19.0,0},
    hConWin=2.7,
    RWin=0.017727777777,
    gWin=0.78,
    UWin=2.1,
    ratioWinConRad=0.09,
    AExt={244.12,416.33,244.12,416.33,208.16},
    hConExt=2.19,
    nExt=1,
    RExt={1.4142107968e-05},
    RExtRem=0.000380773816236,
    CExt={492976267.489},
    AInt=5862.5,
    hConInt=2.27,
    nInt=1,
    RInt={1.13047235829e-05},
    CInt={1402628013.98},
    AFloor=0,
    hConFloor=0,
    nFloor=1,
    RFloor={0.001},
    RFloorRem=0.001,
    CFloor={0.001},
    ARoof=0,
    hConRoof=0,
    nRoof=1,
    RRoof={0.001},
    RRoofRem=0.001,
    CRoof={0.001},
    nOrientationsRoof=1,
    tiltRoof={0},
    aziRoof={0},
    wfRoof={1},
    aRoof=0.7,
    aExt=0.7,
    TSoil=283.15,
    hConWallOut=20.0,
    hRadWall=5,
    hConWinOut=20.0,
    hConRoofOut=20,
    hRadRoof=5,
    tiltExtWalls={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949,0},
    aziExtWalls={0,1.5707963267949,3.1415926535898,4.7123889803847,0},
    wfWall={0.2,0.2,0.2,0.2,0.1},
    wfWin={0.25,0.25,0.25,0.25,0},
    wfGro=0.1,
    specificPeople=1/14,
    activityDegree=1.2,
    fixedHeatFlowRatePersons=70,
    ratioConvectiveHeatPeople=0.5,
    internalGainsMoistureNoPeople=0.5,
    internalGainsMachinesSpecific=7.0,
    ratioConvectiveHeatMachines=0.6,
    lightingPowerSpecific=12.5,
    ratioConvectiveHeatLighting=0.6,
    useConstantACHrate=False,
    baseACH=0.2,
    maxUserACH=1,
    maxOverheatingACH={3.0,2.0},
    maxSummerACH={1.0,273.15 + 10,273.15 + 17},
    winterReduction={0.2,273.15,273.15 + 10},
    withAHU=True,
    minAHU=0,
    maxAHU=12,
    maxIrr = {100,100,100,100,0},
    shadingFactor = {0.7,0.7,0.7,0.7,0},
    hHeat=167500,
    lHeat=0,
    KRHeat=1000,
    TNHeat=1,
    HeaterOn=False,
    hCool=0,
    lCool=-1,
    heaLoadFacOut=0,
    heaLoadFacGrd=0,
    KRCool=1000,
    TNCool=1,
    CoolerOn=False,
    TThresholdHeater=273.15 + 15,
    TThresholdCooler=273.15 + 22,
    withIdealThresholds=False))
    """
    return {
        "classes": ["Space"],
        "library": "reduced_order",
        "parameter_processing": {
            "function": "modify_alias",
            "parameter": {"volume": "	VAir"},
        },
        "ports": [
            {
                "names": ["weaBus"],
                "targets": ["Weather"],
                "medium": "data",
                "flow": "undirected",
            },
            {
                "names": ["TAir"],
                "targets": ["DataBus"],
                "medium": "data",
                "flow": "outlet",
            },
            {
                "names": ["intGains"],
                "targets": ["BaseOccupancy"],
                "medium": "data",
                "flow": "undirected",
            },
        ],
        "template": zone_space_template,
        "variant": "default",
    }


@pytest.fixture
def reduced_order_occupancy():
    zone_space_template = """
 Modelica.Blocks.Sources.CombiTimeTable {{element.name}}(
    extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
    tableName = "UserProfiles",
    columns = {2, 3, 4},
    tableOnFile=False,
    table=[0,0,0.1,0,0; 3540,0,0.1,0,0; 3600,0,0.1,0,0; 7140,0,0.1,0,0; 7200,0,
        0.1,0,0; 10740,0,0.1,0,0; 10800,0,0.1,0,0; 14340,0,0.1,0,0; 14400,0,0.1,
        0,0; 17940,0,0.1,0,0; 18000,0,0.1,0,0; 21540,0,0.1,0,0; 21600,0,0.1,0,0;
        25140,0,0.1,0,0; 25200,0,0.1,0,0; 28740,0,0.1,0,0; 28800,0,0.1,0,0;
        32340,0,0.1,0,0; 32400,0.6,0.6,1,1; 35940,0.6,0.6,1,1; 36000,1,1,1,1;
        39540,1,1,1,1; 39600,0.4,0.4,1,1; 43140,0.4,0.4,1,1; 43200,0,0.1,0,0;
        46740,0,0.1,0,0; 46800,0,0.1,0,0; 50340,0,0.1,0,0; 50400,0.6,0.6,1,1;
        53940,0.6,0.6,1,1; 54000,1,1,1,1; 57540,1,1,1,1; 57600,0.4,0.4,1,1;
        61140,0.4,0.4,1,1; 61200,0,0.1,0,0; 64740,0,0.1,0,0; 64800,0,0.1,0,0;
        68340,0,0.1,0,0; 68400,0,0.1,0,0; 71940,0,0.1,0,0; 72000,0,0.1,0,0;
        75540,0,0.1,0,0; 75600,0,0.1,0,0; 79140,0,0.1,0,0; 79200,0,0.1,0,0;
        82740,0,0.1,0,0; 82800,0,0.1,0,0; 86340,0,0.1,0,0; 86400,0,0.1,0,0;
        89940,0,0.1,0,0; 90000,0,0.1,0,0; 93540,0,0.1,0,0; 93600,0,0.1,0,0;
        97140,0,0.1,0,0; 97200,0,0.1,0,0; 100740,0,0.1,0,0; 100800,0,0.1,0,0;
        104340,0,0.1,0,0; 104400,0,0.1,0,0; 107940,0,0.1,0,0; 108000,0,0.1,0,0;
        111540,0,0.1,0,0; 111600,0,0.1,0,0; 115140,0,0.1,0,0; 115200,0,0.1,0,0;
        118740,0,0.1,0,0; 118800,0.6,0.6,1,1; 122340,0.6,0.6,1,1; 122400,1,1,1,
        1; 125940,1,1,1,1; 126000,0.4,0.4,1,1; 129540,0.4,0.4,1,1; 129600,0,0.1,
        0,0; 133140,0,0.1,0,0; 133200,0,0.1,0,0; 136740,0,0.1,0,0; 136800,0.6,
        0.6,1,1; 140340,0.6,0.6,1,1; 140400,1,1,1,1; 143940,1,1,1,1; 144000,0.4,
        0.4,1,1; 147540,0.4,0.4,1,1; 147600,0,0.1,0,0; 151140,0,0.1,0,0; 151200,
        0,0.1,0,0; 154740,0,0.1,0,0; 154800,0,0.1,0,0; 158340,0,0.1,0,0; 158400,
        0,0.1,0,0; 161940,0,0.1,0,0; 162000,0,0.1,0,0; 165540,0,0.1,0,0; 165600,
        0,0.1,0,0; 169140,0,0.1,0,0; 169200,0,0.1,0,0; 172740,0,0.1,0,0; 172800,
        0,0.1,0,0; 176340,0,0.1,0,0; 176400,0,0.1,0,0; 179940,0,0.1,0,0; 180000,
        0,0.1,0,0; 183540,0,0.1,0,0; 183600,0,0.1,0,0; 187140,0,0.1,0,0; 187200,
        0,0.1,0,0; 190740,0,0.1,0,0; 190800,0,0.1,0,0; 194340,0,0.1,0,0; 194400,
        0,0.1,0,0; 197940,0,0.1,0,0; 198000,0,0.1,0,0; 201540,0,0.1,0,0; 201600,
        0,0.1,0,0; 205140,0,0.1,0,0; 205200,0.6,0.6,1,1; 208740,0.6,0.6,1,1;
        208800,1,1,1,1; 212340,1,1,1,1; 212400,0.4,0.4,1,1; 215940,0.4,0.4,1,1;
        216000,0,0.1,0,0; 219540,0,0.1,0,0; 219600,0,0.1,0,0; 223140,0,0.1,0,0;
        223200,0.6,0.6,1,1; 226740,0.6,0.6,1,1; 226800,1,1,1,1; 230340,1,1,1,1;
        230400,0.4,0.4,1,1; 233940,0.4,0.4,1,1; 234000,0,0.1,0,0; 237540,0,0.1,
        0,0; 237600,0,0.1,0,0; 241140,0,0.1,0,0; 241200,0,0.1,0,0; 244740,0,0.1,
        0,0; 244800,0,0.1,0,0; 248340,0,0.1,0,0; 248400,0,0.1,0,0; 251940,0,0.1,
        0,0; 252000,0,0.1,0,0; 255540,0,0.1,0,0; 255600,0,0.1,0,0; 259140,0,0.1,
        0,0; 259200,0,0.1,0,0; 262740,0,0.1,0,0; 262800,0,0.1,0,0; 266340,0,0.1,
        0,0; 266400,0,0.1,0,0; 269940,0,0.1,0,0; 270000,0,0.1,0,0; 273540,0,0.1,
        0,0; 273600,0,0.1,0,0; 277140,0,0.1,0,0; 277200,0,0.1,0,0; 280740,0,0.1,
        0,0; 280800,0,0.1,0,0; 284340,0,0.1,0,0; 284400,0,0.1,0,0; 287940,0,0.1,
        0,0; 288000,0,0.1,0,0; 291540,0,0.1,0,0; 291600,0.6,0.6,1,1; 295140,0.6,
        0.6,1,1; 295200,1,1,1,1; 298740,1,1,1,1; 298800,0.4,0.4,1,1; 302340,0.4,
        0.4,1,1; 302400,0,0.1,0,0; 305940,0,0.1,0,0; 306000,0,0.1,0,0; 309540,0,
        0.1,0,0; 309600,0.6,0.6,1,1; 313140,0.6,0.6,1,1; 313200,1,1,1,1; 316740,
        1,1,1,1; 316800,0.4,0.4,1,1; 320340,0.4,0.4,1,1; 320400,0,0.1,0,0;
        323940,0,0.1,0,0; 324000,0,0.1,0,0; 327540,0,0.1,0,0; 327600,0,0.1,0,0;
        331140,0,0.1,0,0; 331200,0,0.1,0,0; 334740,0,0.1,0,0; 334800,0,0.1,0,0;
        338340,0,0.1,0,0; 338400,0,0.1,0,0; 341940,0,0.1,0,0; 342000,0,0.1,0,0;
        345540,0,0.1,0,0; 345600,0,0.1,0,0; 349140,0,0.1,0,0; 349200,0,0.1,0,0;
        352740,0,0.1,0,0; 352800,0,0.1,0,0; 356340,0,0.1,0,0; 356400,0,0.1,0,0;
        359940,0,0.1,0,0; 360000,0,0.1,0,0; 363540,0,0.1,0,0; 363600,0,0.1,0,0;
        367140,0,0.1,0,0; 367200,0,0.1,0,0; 370740,0,0.1,0,0; 370800,0,0.1,0,0;
        374340,0,0.1,0,0; 374400,0,0.1,0,0; 377940,0,0.1,0,0; 378000,0.6,0.6,1,
        1; 381540,0.6,0.6,1,1; 381600,1,1,1,1; 385140,1,1,1,1; 385200,0.4,0.4,1,
        1; 388740,0.4,0.4,1,1; 388800,0,0.1,0,0; 392340,0,0.1,0,0; 392400,0,0.1,
        0,0; 395940,0,0.1,0,0; 396000,0.6,0.6,1,1; 399540,0.6,0.6,1,1; 399600,1,
        1,1,1; 403140,1,1,1,1; 403200,0.4,0.4,1,1; 406740,0.4,0.4,1,1; 406800,0,
        0.1,0,0; 410340,0,0.1,0,0; 410400,0,0.1,0,0; 413940,0,0.1,0,0; 414000,0,
        0.1,0,0; 417540,0,0.1,0,0; 417600,0,0.1,0,0; 421140,0,0.1,0,0; 421200,0,
        0.1,0,0; 424740,0,0.1,0,0; 424800,0,0.1,0,0; 428340,0,0.1,0,0; 428400,0,
        0.1,0,0; 431940,0,0.1,0,0; 432000,0,0,0,0; 435540,0,0,0,0; 435600,0,0,0,
        0; 439140,0,0,0,0; 439200,0,0,0,0; 442740,0,0,0,0; 442800,0,0,0,0;
        446340,0,0,0,0; 446400,0,0,0,0; 449940,0,0,0,0; 450000,0,0,0,0; 453540,
        0,0,0,0; 453600,0,0,0,0; 457140,0,0,0,0; 457200,0,0,0,0; 460740,0,0,0,0;
        460800,0,0,0,0; 464340,0,0,0,0; 464400,0,0,0,0; 467940,0,0,0,0; 468000,
        0,0,0,0; 471540,0,0,0,0; 471600,0,0,0,0; 475140,0,0,0,0; 475200,0,0,0,0;
        478740,0,0,0,0; 478800,0,0,0,0; 482340,0,0,0,0; 482400,0,0,0,0; 485940,
        0,0,0,0; 486000,0,0,0,0; 489540,0,0,0,0; 489600,0,0,0,0; 493140,0,0,0,0;
        493200,0,0,0,0; 496740,0,0,0,0; 496800,0,0,0,0; 500340,0,0,0,0; 500400,
        0,0,0,0; 503940,0,0,0,0; 504000,0,0,0,0; 507540,0,0,0,0; 507600,0,0,0,0;
        511140,0,0,0,0; 511200,0,0,0,0; 514740,0,0,0,0; 514800,0,0,0,0; 518340,
        0,0,0,0; 518400,0,0,0,0; 521940,0,0,0,0; 522000,0,0,0,0; 525540,0,0,0,0;
        525600,0,0,0,0; 529140,0,0,0,0; 529200,0,0,0,0; 532740,0,0,0,0; 532800,
        0,0,0,0; 536340,0,0,0,0; 536400,0,0,0,0; 539940,0,0,0,0; 540000,0,0,0,0;
        543540,0,0,0,0; 543600,0,0,0,0; 547140,0,0,0,0; 547200,0,0,0,0; 550740,
        0,0,0,0; 550800,0,0,0,0; 554340,0,0,0,0; 554400,0,0,0,0; 557940,0,0,0,0;
        558000,0,0,0,0; 561540,0,0,0,0; 561600,0,0,0,0; 565140,0,0,0,0; 565200,
        0,0,0,0; 568740,0,0,0,0; 568800,0,0,0,0; 572340,0,0,0,0; 572400,0,0,0,0;
        575940,0,0,0,0; 576000,0,0,0,0; 579540,0,0,0,0; 579600,0,0,0,0; 583140,
        0,0,0,0; 583200,0,0,0,0; 586740,0,0,0,0; 586800,0,0,0,0; 590340,0,0,0,0;
        590400,0,0,0,0; 593940,0,0,0,0; 594000,0,0,0,0; 597540,0,0,0,0; 597600,
        0,0,0,0; 601140,0,0,0,0; 601200,0,0,0,0; 604740,0,0,0,0])
    "Table with profiles for internal gains"
    """
    return {
        "classes": ["Occupancy"],
        "library": "reduced_order",
        "ports": [
            {
                "names": ["y"],
                "targets": ["Space"],
                "medium": "data",
                "flow": "undirected",
            }
        ],
        "template": zone_space_template,
        "variant": "default",
    }


@pytest.fixture
def data_bus():
    template = """
    model DataServerReducedOrder
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;
  Modelica.Blocks.Interfaces.RealInput[{{ element.spaces | length }}] u
    {% raw %}annotation (Placement(transformation(extent={{-142,20},{-102,60}})));{% endraw %}
  Modelica.Blocks.Routing.RealPassThrough[{{ element.spaces | length }}] TRoo
    {% raw %}annotation (Placement(transformation(extent={{-46,30},{-26,50}})));{% endraw %}

  {{ bus_template }}

  {% for input in element.non_connected_ports %}
    {{ input.input_model | safe }}
  {% endfor %}

  {% if element.validation_data.data %}
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
      tableOnFile = False,
      table = [{{ element.validation_data.data | safe }}]
    ) {% raw %}annotation (
      Placement(transformation(extent={{-4,-34},{40,10}}))
    );{% endraw %}
  {% endif %}

equation

  {% if element.validation_data.data %}
    {% for index, column in element.validation_data.columns | enumerate %}
      connect(dataBus.{{ column }}, combiTimeTable.y[{{ index + 1 }}]);
    {% endfor %}
  {% endif %}

  {% for index, space in element.spaces | enumerate %}
    connect(u[{{ index + 1 }}], TRoo[{{ index + 1 }}].u);
  {% endfor %}

  {{ bus_ports | safe }}

  {% for input in element.non_connected_ports %}
    connect(dataBus.{{ input.name }}{{ input.target.value }}, 
            {{ input.name }}{{ input.target.evaluated_element | capitalize }}.y);
  {% endfor %}

end DataServerReducedOrder;
    """
    return {
        "classes": ["DataBus"],
        "component_template": {
            "bus": {
                "real_outputs": [
                    {
                        "component": "TRoo",
                        "name": "TZon",
                        "port": "y",
                        "target": {"main": "element.spaces"},
                    }
                ]
            },
            "category": "control",
            "template": template,
        },
        "figures": [],
        "library": "default",
        "parameter_processing": {"function": "default_parameters"},
        "ports": [
            {
                "multi_connection": True,
                "names": ["u"],
                "targets": ["Space"],
                "flow": "inlet",
                "medium": "data",
            }
        ],
        "template": "    {{package_name}}.Components.BaseClasses.DataServerReducedOrder\n        {{ element.name }} (redeclare package\n          Medium = Medium)",
        "variant": "default",
    }


@pytest.mark.parametrize(
    "simple_space_template, weather",
    [(reduced_order(), reduced_order_weather())],
    indirect=True,
)
def test_use_new_rc_space_library(
    simple_space_template: Space, weather: Weather, reduced_order_occupancy, data_bus
):
    simple_space_template.occupancy = Occupancy(
        name="occupancy_0",
        libraries_data=[LibraryData.model_validate(reduced_order_occupancy)],
    )
    library = Library(
        name="reduced_order",
        merged_external_boundaries=False,
        templates=Templates(construction="", glazing="", main=""),
        medium=MediumTemplate(air="AixLib.Media.Air", water="AixLib.Media.Water"),
    )
    network = Network(name="house_model", library=library)
    network.add_boiler_plate_spaces([simple_space_template], weather=weather)
    # network.connect()
    # simple_space_template.get_neighhors(network.graph)
    # model = simple_space_template.model(network)
    model = network.model(
        include_container=True,
        data_bus=DataBus(libraries_data=[LibraryData.model_validate(data_bus)]),
    )
    assert model
    assert {
        c.equation_view()
        for c in network.containers.get_container("envelope").connections
    } == {
        ("occupancy_0.y", "space_1.intGains"),
        ("space_1.TAir", "y"),
        ("space_1.weaBus", "weather_1.weaBus"),
    }
    assert {
        c.equation_view() for c in network.containers.get_container("bus").connections
    } == {("data_bus.u[1]", "u[1]")}
