"""Glazing properties derived from the layer data of the YAML description.

The solar optics must equal what Buildings computes itself from the same data, so that IDEAS and
AixLib see the glazing Buildings sees. The thermal properties follow EN 673 (U-value) and EN 410
(g-value).
"""

import json
import re
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from tests.constructions.constructions import GasMaterials
from trano.elements.construction import Gas, GasLayer, Glass, GlassLayer, GlassMaterial
from trano.elements.glazing import (
    EN673_MEAN_TEMPERATURE,
    EXTERIOR_SURFACE_COEFFICIENT,
    STEFAN_BOLTZMANN,
    GlazingOptics,
    PaneOptics,
    interior_surface_coefficient,
)
from trano.elements.jinja import compile_template
from trano.elements.library.library import Library

REFERENCE: dict[str, Any] = json.loads(
    Path(__file__).parent.joinpath("resources", "buildings_glazing_reference.json").read_text()
)
GLASS_CONDUCTIVITY = 1.0  # [W/(m.K)]
UNCOATED_EMISSIVITY = 0.837  # corrected emissivity of uncoated soda lime glass (EN 673)


def clear_glass(emissivity: float = UNCOATED_EMISSIVITY) -> GlassMaterial:
    return GlassMaterial(
        name="clear",
        thermal_conductivity=GLASS_CONDUCTIVITY,
        density=2500,
        specific_heat_capacity=840,
        solar_transmittance=[0.834],
        solar_reflectance_outside_facing=[0.075],
        solar_reflectance_room_facing=[0.075],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=emissivity,
        infrared_absorptivity_room_facing=emissivity,
    )


def glazing(panes: int, gap: float = 0.016, gas: Gas = GasMaterials.air, pane: float = 0.004) -> Glass:
    """Glazing of identical clear panes separated by identical gas gaps."""
    layers: list[GlassLayer | GasLayer] = [GlassLayer(thickness=pane, material=clear_glass())]
    for _ in range(panes - 1):
        layers += [GasLayer(thickness=gap, material=gas), GlassLayer(thickness=pane, material=clear_glass())]
    return Glass(name=f"glazing_{panes}", u_value_frame=1.4, layers=layers)


@pytest.mark.parametrize("case", REFERENCE["cases"], ids=lambda case: case["name"])
def test_solar_optics_equal_the_buildings_window_model(case: dict[str, Any]) -> None:
    optics = GlazingOptics.from_panes([PaneOptics.from_normal_incidence(**pane) for pane in case["panes"]])

    np.testing.assert_allclose(optics.transmittance, case["transmittance"], rtol=0, atol=1e-12)
    np.testing.assert_allclose(optics.absorptances, case["absorptances"], rtol=0, atol=1e-12)


@pytest.mark.parametrize(
    ("glass", "declared_u_value"),
    [
        (glazing(1), 5.8),
        (glazing(2, gap=0.006), 3.3),
        (glazing(2, gap=0.016), 2.7),
        (glazing(2, gap=0.016, gas=GasMaterials.argon), 2.6),
    ],
    ids=["4", "4-6-4 air", "4-16-4 air", "4-16-4 argon"],
)
def test_u_value_matches_the_en_673_declared_value(glass: Glass, declared_u_value: float) -> None:
    # EN 673 declares U-values rounded to 0.1 W/(m2.K).
    assert round(glass.properties.u_value, 1) == declared_u_value


def test_low_emissivity_coating_only_reduces_the_radiation_across_the_gap() -> None:
    low_e = clear_glass().model_copy(update={"infrared_absorptivity_outside_facing": 0.1})  # coating on face 3
    coated = Glass(
        name="low_e",
        u_value_frame=1.4,
        layers=[
            GlassLayer(thickness=0.004, material=clear_glass()),
            GasLayer(thickness=0.016, material=GasMaterials.air),
            GlassLayer(thickness=0.004, material=low_e),
        ],
    )

    def radiation(emissivity_a: float, emissivity_b: float) -> float:
        return 4 * STEFAN_BOLTZMANN * EN673_MEAN_TEMPERATURE**3 / (1 / emissivity_a + 1 / emissivity_b - 1)

    uncoated_gap = 1 / glazing(2).properties.gap_resistances[0]
    coated_gap = 1 / coated.properties.gap_resistances[0]
    assert uncoated_gap - coated_gap == pytest.approx(
        radiation(UNCOATED_EMISSIVITY, UNCOATED_EMISSIVITY) - radiation(UNCOATED_EMISSIVITY, 0.1)
    )
    assert coated.properties.u_value < glazing(2).properties.u_value


def test_single_pane_releases_its_absorbed_share_by_the_surface_resistance_ratio() -> None:
    # EN 410 with the pane as one isothermal node: q_i = alpha_e * R_out / R_total.
    properties = glazing(1, pane=0.004).properties
    absorptance = 1 - 0.834 - 0.075
    resistance_to_outside = 1 / EXTERIOR_SURFACE_COEFFICIENT + 0.004 / GLASS_CONDUCTIVITY / 2
    total_resistance = (
        1 / EXTERIOR_SURFACE_COEFFICIENT
        + 0.004 / GLASS_CONDUCTIVITY
        + 1 / interior_surface_coefficient(UNCOATED_EMISSIVITY)
    )

    assert properties.solar_transmittance == pytest.approx(0.834)
    assert properties.secondary_heat_transfer == pytest.approx(absorptance * resistance_to_outside / total_resistance)
    assert properties.g_value == pytest.approx(properties.solar_transmittance + properties.secondary_heat_transfer)


def test_g_value_decreases_with_the_number_of_panes() -> None:
    single, double, triple = (glazing(panes).properties for panes in (1, 2, 3))

    assert single.g_value > double.g_value > triple.g_value
    assert double.g_value == pytest.approx(0.76, abs=0.01)  # clear double glazing


@pytest.mark.parametrize(
    "layers",
    [
        [GasLayer(thickness=0.016, material=GasMaterials.air)],
        [GlassLayer(thickness=0.004, material=clear_glass()), GlassLayer(thickness=0.004, material=clear_glass())],
        [
            GlassLayer(thickness=0.004, material=clear_glass()),
            GasLayer(thickness=0.016, material=GasMaterials.air),
        ],
        [],
    ],
    ids=["gas only", "two panes without gap", "ends with gas", "empty"],
)
def test_glazing_layers_must_alternate_glass_and_gas(layers: list[GlassLayer | GasLayer]) -> None:
    with pytest.raises(ValueError, match="must alternate glass panes and gas gaps"):
        Glass(name="invalid", u_value_frame=1.4, layers=layers)


def ideas_glazing_record(glass: Glass) -> str:
    template = Library.from_configuration("IDEAS").templates.glazing
    return compile_template("{% import 'macros.jinja2' as macros %}" + template).render(
        construction=glass, package_name="building"
    )


def matrix(record: str, name: str) -> list[list[float]]:
    match = re.search(rf"{name}=\[(.*?)\]", record, re.DOTALL)
    assert match, f"{name} not rendered"
    return [[float(value) for value in row.split(",")] for row in match.group(1).split(";")]


@pytest.mark.parametrize("panes", [1, 2, 3])
def test_ideas_glazing_record_is_sized_by_its_layers(panes: int) -> None:
    glass = glazing(panes)
    record = ideas_glazing_record(glass)
    layers = 2 * panes - 1

    assert f"nLay={layers}" in record
    assert len(re.findall(r"building\.Data\.Materials\.\w+\s*\(d=", record)) == layers
    transmittance, absorptance = matrix(record, "SwTrans"), matrix(record, "SwAbs")
    assert [len(row) for row in transmittance] == [2] * 10
    assert [len(row) for row in absorptance] == [layers + 1] * 10
    assert [row[0] for row in absorptance] == [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]
    gas_columns = range(2, layers + 1, 2)  # after the angle column, the layers alternate glass and gas
    assert all(row[column] == 0 for row in absorptance for column in gas_columns)
    absorbed_diffuse = re.search(r"SwAbsDif=\{(.*?)\}", record)
    assert absorbed_diffuse and len(absorbed_diffuse.group(1).split(",")) == layers
    assert f"U_value={round(glass.properties.u_value, 4)}" in record
    assert f"g_value={round(glass.properties.g_value, 4)}" in record
