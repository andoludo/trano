from pathlib import Path
from typing import Callable, List, Optional

from pydantic import Field

from neosim.controller.parser import ControllerBus
from neosim.models.elements.base import (
    AvailableLibraries,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from neosim.models.elements.bus import DataBus
from neosim.models.elements.controls.base import Control
from neosim.models.elements.system import System

dynamic_ahu_controller_template = DynamicComponentTemplate(
    template="""model AhuControl{{ element.name | capitalize}}
  parameter Real VUncDesOutAir_flow=0.03;
  parameter Real VDesTotOutAir_flow=0.03;
      Buildings.Controls.OBC.ASHRAE.G36.AHUs.MultiZone.VAV.Controller
        mulAHUCon(
        eneStd=Buildings.Controls.OBC.ASHRAE.G36.Types.EnergyStandard.ASHRAE90_1,
        venStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,
        ashCliZon=Buildings.Controls.OBC.ASHRAE.G36.Types.ASHRAEClimateZone.Zone_1A,
        have_frePro=false,
        minOADes=Buildings.Controls.OBC.ASHRAE.G36.Types.OutdoorAirSection.DedicatedDampersAirflow,
        buiPreCon=Buildings.Controls.OBC.ASHRAE.G36.Types.PressureControl.BarometricRelief,
        ecoHigLimCon=Buildings.Controls.OBC.ASHRAE.G36.Types.ControlEconomizer.FixedDryBulb,
        cooCoi=Buildings.Controls.OBC.ASHRAE.G36.Types.CoolingCoil.None,
        heaCoi=Buildings.Controls.OBC.ASHRAE.G36.Types.HeatingCoil.None,
        have_perZonRehBox=false, VUncDesOutAir_flow = VUncDesOutAir_flow, VDesTotOutAir_flow = VDesTotOutAir_flow)
        {% raw %}annotation (Placement(transformation(extent={{-12,-14},{28,74}})));{% endraw %}
{{bus_template}}

  Buildings.Controls.OBC.ASHRAE.G36.AHUs.MultiZone.VAV.SetPoints.OutdoorAirFlow.ASHRAE62_1.SumZone
    sumZon(nZon={{element.vavs | length }}, nGro=1,     final zonGroMat=[1],
    final zonGroMatTra=[1])
    {% raw %}annotation (Placement(transformation(extent={{-72,32},{-52,52}})));{% endraw %}
      Buildings.Controls.OBC.CDL.Integers.MultiSum preRetReq(final nin={{element.vavs | length }})
    {% raw %}annotation (Placement(transformation(extent={{-72,80},{-60,92}})));{% endraw %}
  Buildings.Controls.OBC.CDL.Integers.MultiSum temResReq(final nin={{element.vavs | length }})
    {% raw %}annotation (Placement(transformation(extent={{-72,56},{-60,68}})));{% endraw %}
equation
{{bus_ports | safe}}
{% raw %}
  connect(sumZon.VSumAdjPopBreZon_flow, mulAHUCon.VSumAdjPopBreZon_flow)
    annotation (Line(points={{-50,50},{-22,50},{-22,55},{-14,55}}, color={0,0,127}));
  connect(sumZon.VSumAdjAreBreZon_flow, mulAHUCon.VSumAdjAreBreZon_flow)
    annotation (Line(points={{-50,46},{-20,46},{-20,53},{-14,53}}, color={0,0,127}));
  connect(sumZon.VSumZonPri_flow, mulAHUCon.VSumZonPri_flow) annotation (Line(
        points={{-50,38},{-38,38},{-38,44},{-14,44},{-14,50}}, color={0,0,127}));
  connect(sumZon.uOutAirFra_max, mulAHUCon.uOutAirFra_max) annotation (Line(
        points={{-50,34},{-34,34},{-34,40},{-20,40},{-20,42},{-14,42},{-14,47}},
        color={0,0,127}));
      connect(temResReq.y, mulAHUCon.uZonTemResReq) annotation (Line(points={{-58.8,
          62},{-58.8,63},{-14,63}}, color={255,127,0}));
  connect(preRetReq.y, mulAHUCon.uZonPreResReq) annotation (Line(points={{-58.8,
          86},{-22,86},{-22,71},{-14,71}}, color={255,127,0}));
        {% endraw %}
end AhuControl{{ element.name  | capitalize}};""",
    category="control",
    bus=ControllerBus.from_configuration(
        Path(__file__).parent.joinpath("config", "ahu.json")
    ),
)


class BaseAhuControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.AhuControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: str = dynamic_ahu_controller_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[System, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class AhuControl(Control):
    spaces: Optional[List[str]] = None
    vavs: Optional[List[str]] = None
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseAhuControl],
        buildings=[BaseAhuControl],
    )
