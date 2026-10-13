"""Ventilation units, PV arrays and the systems left out by the multilayer models of the ``mpc`` library."""

import warnings
from pathlib import Path

import numpy as np
import pytest
import rumoca

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.mpc.components import ConnectedModel, Instance, TopInput, library_source
from trano.mpc.components.envelope import UnmodelledSystemWarning, multilayer_model
from trano.topology import Network

ROOT = Path(__file__).parents[1]
RENOVATED = ROOT / "examples" / "residential_after_renovation.yaml"


def _exports_connected_models() -> bool:
    from tests.test_mpc_connected import _exports_connected_models as probe

    return probe()


needs_connected_export = pytest.mark.skipif(
    not _exports_connected_models(), reason="the CasADi export of connected models needs rumoca >= 0.10.3"
)


def _network(path: Path) -> Network:
    library = Library.from_configuration("mpc").model_copy(update={"connected": True, "envelope": "multilayer"})
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=library)


def _export(model: ConnectedModel) -> object:
    library = f"package Trano\npackage MPC\nconnector RealInput = input Real;\n{library_source()}\nend MPC;\nend Trano;"
    return rumoca.Session().loads(f"package T\n{library}\n{model.render()}\nend T;\n", model="T.M").to_casadi()


# ------------------------------------------------------------------------------------------- generator
def test_the_renovated_house_has_its_ventilation_unit_and_pv_array() -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("error", UnmodelledSystemWarning)  # everything of the YAML is modelled
        model = multilayer_model(_network(RENOVATED))
    unit = next(instance for instance in model.instances if instance.component == "Ventilation.HeatRecoveryUnit")
    assert unit.parameters["n"] == 6
    assert unit.parameters["eps"] == pytest.approx(0.8)
    assert sum(unit.parameters["mNom"]) == pytest.approx(0.05, rel=0.01)  # type: ignore[arg-type]
    assert unit.inputs == {"y": "ahu_001_y", "TOut": "TOut"}
    assert ("ahu_001.zone[6]", "living_001_air.port") in model.connections
    array = next(instance for instance in model.instances if instance.component == "Electrical.Photovoltaic")
    assert array.inputs == {"HDir": "HDir_azi0_til35", "HDif": "HDif_azi0_til35"}
    assert {"HDir_azi0_til35", "ahu_001_y"} <= set(model.input_names)
    assert {"ahu_001.EFan", "pv_001.EPro"} <= set(model.state_names)


def test_systems_left_out_are_reported() -> None:
    with pytest.warns(UnmodelledSystemWarning, match="battery|chiller|dhw|ev_charger"):
        multilayer_model(_network(ROOT / "tests" / "models" / "house_mpc_systems.yaml"))


# --------------------------------------------------------------------------------------------- physics
@needs_connected_export
def test_the_heat_recovery_unit_supplies_recovered_air() -> None:
    """Two zones at 20 and 22 degC, 0 degC outside: supply at TOut + eps*(TExt - TOut) + dTFan."""
    model = ConnectedModel(
        name="M",
        inputs=[TopInput(name="TOut", unit="K"), TopInput(name="y")],
        instances=[
            Instance(name="z1", component="Thermal.HeatCapacitor", parameters={"C": 1e5}),
            Instance(name="z2", component="Thermal.HeatCapacitor", parameters={"C": 2e5}),
            Instance(
                name="ahu",
                component="Ventilation.HeatRecoveryUnit",
                parameters={"n": 2, "mNom": [0.01, 0.03], "eps": 0.8, "dTFan": 0.4, "PNom": 30.0},
                inputs={"y": "y", "TOut": "TOut"},
            ),
        ],
        connections=[("ahu.zone[1]", "z1.port"), ("ahu.zone[2]", "z2.port")],
    )
    export = _export(model)
    t1, t2, t_out, y = 293.15, 295.15, 273.15, 0.5
    xdot = np.array(export.rhs(0, [t1, t2, 0.0], [t_out, y], export.default_parameters)).ravel()  # type: ignore[attr-defined]
    extract = (0.01 * t1 + 0.03 * t2) / 0.04
    supply = t_out + 0.8 * (extract - t_out) + 0.4
    assert xdot[0] * 1e5 == pytest.approx(-y * 0.01 * 1006 * (t1 - supply))
    assert xdot[1] * 2e5 == pytest.approx(-y * 0.03 * 1006 * (t2 - supply))
    assert xdot[2] * 3.6e6 == pytest.approx(y * 30.0)  # fan electricity


@needs_connected_export
def test_the_pv_array_follows_buildings() -> None:
    model = ConnectedModel(
        name="M",
        inputs=[TopInput(name="HDir", unit="W/m2"), TopInput(name="HDif", unit="W/m2")],
        instances=[
            Instance(
                name="pv",
                component="Electrical.Photovoltaic",
                parameters={"A": 21.0, "eta": 0.2},
                inputs={"HDir": "HDir", "HDif": "HDif"},
            )
        ],
        connections=[],
    )
    export = _export(model)
    xdot = np.array(export.rhs(0, [0.0], [600.0, 150.0], export.default_parameters)).ravel()  # type: ignore[attr-defined]
    assert xdot[0] * 3.6e6 == pytest.approx(21.0 * 0.9 * 0.2 * 0.9 * 750.0)  # PVSimpleOriented: A fAct eta eta_DCAC G
