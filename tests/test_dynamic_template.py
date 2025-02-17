import pytest

from trano.elements import VAV, VAVControl
from trano.elements.bus import get_non_connected_ports
from trano.elements.library.components import COMPONENTS

from trano.elements.library.library import Library, AvailableLibraries


@pytest.fixture
def vav_library_data():
    libraries_data = {
        "components": [{
            "classes": [
                "VAVControl"
            ],
            "component_template": {
                "bus": {
                    "boolean_inputs": [
                        {
                            "component": "rehBoxCon",
                            "name": "u1Win",
                            "port": "u1Win",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "u1Occ",
                            "port": "u1Occ",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "uHeaOff",
                            "port": "uHeaOff",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "u1Fan",
                            "port": "u1Fan",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "u1HotPla",
                            "port": "u1HotPla",
                            "target": {
                                "main": "element.space_name"
                            }
                        }
                    ],
                    "integer_inputs": [
                        {
                            "component": "rehBoxCon",
                            "default": 1,
                            "name": "uOpeMod",
                            "port": "uOpeMod",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "oveFloSet",
                            "port": "oveFloSet",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "oveDamPos",
                            "port": "oveDamPos",
                            "target": {
                                "main": "element.space_name"
                            }
                        }
                    ],
                    "integer_outputs": [
                        {
                            "component": "rehBoxCon",
                            "name": "yZonTemResReq",
                            "port": "yZonTemResReq",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yZonPreResReq",
                            "port": "yZonPreResReq",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yHeaValResReq",
                            "port": "yHeaValResReq",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yHotWatPlaReq",
                            "port": "yHotWatPlaReq",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yLowFloAla",
                            "port": "yLowFloAla",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yFloSenAla",
                            "port": "yFloSenAla",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yLeaDamAla",
                            "port": "yLeaDamAla",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yLeaValAla",
                            "port": "yLeaValAla",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yLowTemAla",
                            "port": "yLowTemAla",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        }
                    ],
                    "real_inputs": [
                        {
                            "component": "rehBoxCon",
                            "name": "TZon",
                            "port": "TZon",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "default": 298.15,
                            "name": "TCooSet",
                            "port": "TCooSet",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "default": 293.15,
                            "name": "THeaSet",
                            "port": "THeaSet",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "ppmCO2Set",
                            "port": "ppmCO2Set",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "ppmCO2",
                            "port": "ppmCO2",
                            "target": {
                                "main": "element.space_name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "TDis",
                            "port": "TDis",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "VDis_flow",
                            "port": "VDis_flow",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "default": 293.15,
                            "name": "TSupSet",
                            "port": "TSupSet",
                            "target": {
                                "main": "element.space_name"
                            }
                        }
                    ],
                    "real_outputs": [
                        {
                            "component": "rehBoxCon",
                            "name": "VSet_flow",
                            "port": "VSet_flow",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yDam",
                            "port": "yDam",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yVal",
                            "port": "yVal",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "VAdjPopBreZon_flow",
                            "port": "VAdjPopBreZon_flow",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "VAdjAreBreZon_flow",
                            "port": "VAdjAreBreZon_flow",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "VMinOA_flow",
                            "port": "VMinOA_flow",
                            "target": {
                                "main": "element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "VZonAbsMin_flow",
                            "port": "VZonAbsMin_flow",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "VZonDesMin_flow",
                            "port": "VZonDesMin_flow",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        },
                        {
                            "component": "rehBoxCon",
                            "name": "yCO2",
                            "port": "yCO2",
                            "target": {
                                "main": "element.controllable_element.name"
                            }
                        }
                    ],
                    "template": "Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(\n  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));"
                },
                "category": "control",
                "template": "model VAVControl{{ element.name | capitalize}}\nBuildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller rehBoxCon(\nvenStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,\nhave_winSen=True,\nhave_occSen=True,\nhave_CO2Sen=True,\nhave_hotWatCoi=True,\nVOccMin_flow=0.003,\nVAreMin_flow=0.003,\nVAreBreZon_flow=0.003,\nVPopBreZon_flow=0.003,\nVMin_flow=0.003,\nVCooMax_flow=0.003,\nVHeaMin_flow=0.003,\nVHeaMax_flow=0.003)\n{% raw %}annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); {% endraw %}\n{{bus_template}}\nequation\n{{bus_ports | safe}}\nend VAVControl{{ element.name  | capitalize}};"
            },
            "figures": [],
            "library": "default",
            "parameter_processing": {
                "function": "default_parameters"
            },
            "ports": [
                {
                    "multi_connection": True,
                    "names": [
                        "dataBus"
                    ],
                    "targets": [
                        "System",
                        "DataBus"
                    ],
                    "use_counter": False,
                                        "medium": "data",
                    "flow": "undirected"
                }
            ],
            "template": "\n    {{package_name}}.Trano.Controls.ventilation.VAVControl{{ element.name | capitalize}}\n    {{ element.name }}",
            "variant": "default"
        },
            {
                "classes": [
                    "VAV"
                ],
                "component_template": {
                    "bus": {
                        "real_inputs": [
                            {
                                "component": "vav",
                                "name": "yDam",
                                "port": "y",
                                "target": {
                                    "main": "element.name"
                                }
                            }
                        ],
                        "real_outputs": [
                            {
                                "component": "vav",
                                "name": "y_actual",
                                "port": "y_actual",
                                "target": {
                                    "main": "element.name"
                                }
                            },
                            {
                                "component": "senVolFlo",
                                "name": "VDis_flow",
                                "port": "V_flow",
                                "target": {
                                    "main": "element.control.name"
                                }
                            },
                            {
                                "component": "senTem",
                                "name": "TDis",
                                "port": "T",
                                "target": {
                                    "main": "element.control.name"
                                }
                            }
                        ]
                    },
                    "category": "ventilation",
                    "template": "\nmodel VAVBox{{ element.name | capitalize}}\nextends {{ package_name }}.Trano.Fluid.Ventilation.PartialVAVBox;\n{{bus_template}}\nequation\n{{bus_ports | safe}}\n end VAVBox{{ element.name | capitalize}};\n "
                },
                "figures": [],
                "ports": [],
                "library": "default",
                "parameter_processing": {
                    "function": "default_parameters"
                },
                "template": "  {{ package_name }}.Trano.\n    Fluid.Ventilation.VAVBox{{ element.name | capitalize }}\n     {{ element.name }}(\n    redeclare package MediumA = Medium,\n    mCooAir_flow_nominal=100*1.2/3600,\n    mHeaAir_flow_nominal=100*1.2/3600,\n    VRoo=100,\n    allowFlowReversal=False,\n    THeaWatInl_nominal=90,\n    THeaWatOut_nominal=60,\n    THeaAirInl_nominal=30,\n    THeaAirDis_nominal=25\n    )",
                "variant": "complex"
            }
        ]
    }
    return libraries_data

def test_available_libraries_vav(vav_library_data):
    vav_component = AvailableLibraries.from_config("VAV", vav_library_data)
    a = 12

def test_dynamic_template_vav(vav_library_data):
    vav_library = AvailableLibraries.from_config("VAV", vav_library_data)
    vav_control_library = AvailableLibraries.from_config("VAVControl", vav_library_data)
    vav = VAV(variant="complex", libraries_data=vav_library)
    vav_control = VAVControl(libraries_data=vav_control_library)
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
    assert vav_control_template == ('model VAVControlVavcontrol_0\n'
 'Buildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller '
 'rehBoxCon(\n'
 'venStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,\n'
 'have_winSen=True,\n'
 'have_occSen=True,\n'
 'have_CO2Sen=True,\n'
 'have_hotWatCoi=True,\n'
 'VOccMin_flow=0.003,\n'
 'VAreMin_flow=0.003,\n'
 'VAreBreZon_flow=0.003,\n'
 'VPopBreZon_flow=0.003,\n'
 'VMin_flow=0.003,\n'
 'VCooMax_flow=0.003,\n'
 'VHeaMin_flow=0.003,\n'
 'VHeaMax_flow=0.003)\n'
 'annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); '
 'Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(\n'
 '  extent={{-120,-18},{-80,22}}), '
 'iconTransformation(extent={{-120,62},{-78,98}})));\n'
 'equation\n'
 'connect(dataBus.TZonSpace_vav, rehBoxCon.TZon);\n'
 'connect(dataBus.TCooSetSpace_vav, rehBoxCon.TCooSet);\n'
 'connect(dataBus.THeaSetSpace_vav, rehBoxCon.THeaSet);\n'
 'connect(dataBus.ppmCO2SetSpace_vav, rehBoxCon.ppmCO2Set);\n'
 'connect(dataBus.ppmCO2Space_vav, rehBoxCon.ppmCO2);\n'
 'connect(dataBus.TSupSetSpace_vav, rehBoxCon.TSupSet);\n'
 'connect(dataBus.uOpeModSpace_vav, rehBoxCon.uOpeMod);\n'
 'connect(dataBus.oveFloSetSpace_vav, rehBoxCon.oveFloSet);\n'
 'connect(dataBus.oveDamPosSpace_vav, rehBoxCon.oveDamPos);\n'
 'connect(dataBus.u1WinSpace_vav, rehBoxCon.u1Win);\n'
 'connect(dataBus.u1OccSpace_vav, rehBoxCon.u1Occ);\n'
 'connect(dataBus.uHeaOffSpace_vav, rehBoxCon.uHeaOff);\n'
 'connect(dataBus.u1FanSpace_vav, rehBoxCon.u1Fan);\n'
 'connect(dataBus.u1HotPlaSpace_vav, rehBoxCon.u1HotPla);\n'
 'connect(dataBus.TDisVavcontrol_0, rehBoxCon.TDis);\n'
 'connect(dataBus.VDis_flowVavcontrol_0, rehBoxCon.VDis_flow);\n'
 'connect(dataBus.VAdjPopBreZon_flowVavcontrol_0, '
 'rehBoxCon.VAdjPopBreZon_flow);\n'
 'connect(dataBus.VAdjAreBreZon_flowVavcontrol_0, '
 'rehBoxCon.VAdjAreBreZon_flow);\n'
 'connect(dataBus.VMinOA_flowVavcontrol_0, rehBoxCon.VMinOA_flow);\n'
 'connect(dataBus.yZonTemResReqVavcontrol_0, rehBoxCon.yZonTemResReq);\n'
 'connect(dataBus.yZonPreResReqVavcontrol_0, rehBoxCon.yZonPreResReq);\n'
 'connect(dataBus.yHeaValResReqVavcontrol_0, rehBoxCon.yHeaValResReq);\n'
 'connect(dataBus.VSet_flowVav_0, rehBoxCon.VSet_flow);\n'
 'connect(dataBus.yDamVav_0, rehBoxCon.yDam);\n'
 'connect(dataBus.yValVav_0, rehBoxCon.yVal);\n'
 'connect(dataBus.VZonAbsMin_flowVav_0, rehBoxCon.VZonAbsMin_flow);\n'
 'connect(dataBus.VZonDesMin_flowVav_0, rehBoxCon.VZonDesMin_flow);\n'
 'connect(dataBus.yCO2Vav_0, rehBoxCon.yCO2);\n'
 'connect(dataBus.yHotWatPlaReqVav_0, rehBoxCon.yHotWatPlaReq);\n'
 'connect(dataBus.yLowFloAlaVav_0, rehBoxCon.yLowFloAla);\n'
 'connect(dataBus.yFloSenAlaVav_0, rehBoxCon.yFloSenAla);\n'
 'connect(dataBus.yLeaDamAlaVav_0, rehBoxCon.yLeaDamAla);\n'
 'connect(dataBus.yLeaValAlaVav_0, rehBoxCon.yLeaValAla);\n'
 'connect(dataBus.yLowTemAlaVav_0, rehBoxCon.yLowTemAla);\n'
 'end VAVControlVavcontrol_0;')
    assert rendered_template == ('model VAVBoxVav_0\n'
 'extends test.Trano.Fluid.Ventilation.PartialVAVBox;\n'
 'Controls.BaseClasses.DataBus dataBus\n'
 '    annotation (Placement(transformation(\n'
 '  extent={{-120,-18},{-80,22}}), '
 'iconTransformation(extent={{-120,62},{-78,98}})));\n'
 'equation\n'
 'connect(dataBus.yDamVav_0, vav.y);\n'
 'connect(dataBus.y_actualVav_0, vav.y_actual);\n'
 'connect(dataBus.VDis_flowVavcontrol_0, senVolFlo.V_flow);\n'
 'connect(dataBus.TDisVavcontrol_0, senTem.T);\n'
 ' end VAVBoxVav_0;\n'
 ' ')


def test_available_libraries():
    COMPONENTS
    a = 12