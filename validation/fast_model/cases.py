"""Cases and parameter changes of the FastModel-against-Buildings study.

A case is a YAML building simulated over the same period with Buildings and with the closed-loop FastModel library.
Its *perturbations* each change one field of the YAML (a material property, an air change rate, the rating of a
system, a set point) and are simulated with both libraries too, so that the response of each library to the same
change can be compared: a reduced model is useful for design and calibration only if a parameter moves its results
the way it moves the results of the detailed model.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
BESTEST_CASES = ROOT / "validation" / "bestest" / "cases"
EXAMPLES = ROOT / "examples"
DAY = 86400
BASE = "base"
ENVELOPE, SYSTEM, USE = "envelope", "system", "use"

Document = dict[str, Any]
Change = Callable[[Document], None]


def _by_id(items: Iterable[Document], id_: str) -> Document:
    for item in items:
        if item.get("id") == id_:
            return item
    raise KeyError(id_)


def _spaces(document: Document, spaces: tuple[str, ...] | None) -> Iterable[Document]:
    return (space for space in document["spaces"] if spaces is None or space["id"] in spaces)


def scale_material(id_: str, name: str, factor: float) -> Change:
    """Multiply a property of a material (``thermal_conductivity``, ``density``, ...)."""

    def change(document: Document) -> None:
        item = _by_id(document["material"], id_)
        item[name] = round(float(item[name]) * factor, 6)

    return change


def set_material(ids: tuple[str, ...], name: str, value: float) -> Change:
    """Set a property of materials (``shortwave_emissivity`` of the outer layers, ...)."""

    def change(document: Document) -> None:
        for id_ in ids:
            _by_id(document["material"], id_)[name] = value

    return change


def scale_glass(id_: str, name: str, factor: float) -> Change:
    """Multiply an optical property of a glass, given per pane as a list (``solar_transmittance``)."""

    def change(document: Document) -> None:
        item = _by_id(document["glass_material"], id_)
        item[name] = [round(float(value) * factor, 6) for value in item[name]]

    return change


def scale_gas(id_: str, name: str, factor: float) -> Change:
    """Multiply a property of the gas of the glazing gaps (``thermal_conductivity``)."""

    def change(document: Document) -> None:
        item = _by_id(document["gas"], id_)
        item[name] = round(float(item[name]) * factor, 6)

    return change


def _gain_value(entry: str) -> float:
    numerator, _, denominator = entry.partition("/")
    return float(numerator) / float(denominator or 1)


def scale_gains(factor: float, spaces: tuple[str, ...] | None = None) -> Change:
    """Multiply the internal gains of the spaces: the radiant, convective and latent rows of ``[120/48; 80/48; 0]``."""

    def change(document: Document) -> None:
        for space in _spaces(document, spaces):
            parameters = space.get("occupancy", {}).get("parameters")
            if parameters is None or "gain" not in parameters:
                continue
            rows = parameters["gain"].strip("[] ").split(";")
            parameters["gain"] = "[" + "; ".join(f"{factor * _gain_value(row.strip()):.6g}" for row in rows) + "]"

    return change


def scale_space_parameter(name: str, factor: float, spaces: tuple[str, ...] | None = None) -> Change:
    """Multiply a parameter of the spaces (``ach``, ``n50``), all of them by default."""

    def change(document: Document) -> None:
        for space in _spaces(document, spaces):
            space["parameters"][name] = round(float(space["parameters"][name]) * factor, 6)

    return change


def scale_emission_parameter(kind: str, name: str, factor: float) -> Change:
    """Multiply a parameter of every emission element of a kind (``radiator``) in every space."""

    def change(document: Document) -> None:
        for space in document["spaces"]:
            for emission in space.get("emissions", []):
                if kind in emission:
                    parameters = emission[kind]["parameters"]
                    parameters[name] = round(float(parameters[name]) * factor, 6)

    return change


def emission_parameter(kind: str, name: str, value: float) -> Change:
    """Set a parameter of every emission element of a kind (``radiator``) in every space."""

    def change(document: Document) -> None:
        for space in document["spaces"]:
            for emission in space.get("emissions", []):
                if kind in emission:
                    emission[kind].setdefault("parameters", {})[name] = value

    return change


def scale_ventilation(factor: float) -> Change:
    """Multiply the nominal flow of the air handling units and of every supply and extract duct of the spaces."""

    def change(document: Document) -> None:
        for system in document["systems"]:
            unit = system.get("air_handling_unit")
            if unit is not None:
                unit["parameters"]["m_flow_nominal"] = round(float(unit["parameters"]["m_flow_nominal"]) * factor, 6)
        for space in document["spaces"]:
            for duct in (*space.get("ventilation_inlets", []), *space.get("ventilation_outlets", [])):
                parameters = duct["duct"]["parameters"]
                parameters["nominal_mass_flow_rate"] = round(float(parameters["nominal_mass_flow_rate"]) * factor, 6)

    return change


def emission_control_parameter(name: str, value: float, spaces: tuple[str, ...] | None = None) -> Change:
    """Set a parameter of the emission controls (the thermostats on the valves) of the spaces."""

    def change(document: Document) -> None:
        for space in _spaces(document, spaces):
            for emission in space.get("emissions", []):
                control = (emission.get("valve") or {}).get("control", {}).get("emission_control")
                if control is not None:
                    control["parameters"][name] = value

    return change


def offset_schedule(table: str, delta: float) -> str:
    """A Modelica day table ``[0, 283.15; 25200, 283.15]`` with ``delta`` added to every value."""
    rows = [row.split(",") for row in table.strip("[] ").split(";")]
    return "[" + "; ".join(f"{float(time):g}, {float(value) + delta:g}" for time, value in rows) + "]"


def ideal_setpoint_offset(name: str, delta: float) -> Change:
    """Shift a set point schedule of the ideal heating and cooling systems by ``delta`` [K]."""

    def change(document: Document) -> None:
        for space in document["spaces"]:
            for emission in space.get("emissions", []):
                if "ideal_heating_cooling" in emission:
                    parameters = emission["ideal_heating_cooling"]["parameters"]
                    parameters[name] = offset_schedule(parameters[name], delta)

    return change


def system_parameter(kind: str, id_: str, name: str, value: float | bool) -> Change:
    """Set a parameter of a system (``boiler``, ``air_handling_unit``, ...) found by its id."""

    def change(document: Document) -> None:
        for system in document["systems"]:
            item = system.get(kind)
            if item is not None and item["id"] == id_:
                item["parameters"][name] = value
                return
        raise KeyError(f"{kind} {id_}")

    return change


@dataclass(frozen=True)
class Perturbation:
    id: str
    description: str
    group: str  # ENVELOPE, SYSTEM or USE
    change: Change
    sweep: str | None = None  # the parameter scaled, for the steps of a sweep (the base case is its factor 1)
    factor: float = 1.0  # the factor of the step


def sweep(
    name: str, description: str, group: str, factors: tuple[float, ...], scale: Callable[[float], Change]
) -> tuple[Perturbation, ...]:
    """Steps of one parameter scaled by several factors, ``<name>_x<factor>``: whether a library follows the other
    over a range of values, not only for one step, and whether its response bends as the other's does."""
    return tuple(
        Perturbation(f"{name}_x{factor:g}", f"{description} x{factor:g}", group, scale(factor), name, factor)
        for factor in factors
    )


@dataclass(frozen=True)
class StudyCase:
    id: str
    path: Path
    days: int
    description: str
    start_day: int = 0
    output_interval: int = 3600  # [s] of the result files; the series are hourly means in any case
    perturbations: tuple[Perturbation, ...] = ()
    trace_windows: tuple[tuple[int, int], ...] = ()  # (first day of the year, days) windows shown in the figures

    @property
    def start_time(self) -> int:
        return self.start_day * DAY

    @property
    def end_time(self) -> int:
        return (self.start_day + self.days) * DAY

    @property
    def variants(self) -> list[str]:
        return [BASE, *(perturbation.id for perturbation in self.perturbations)]

    def perturbation(self, variant: str) -> Perturbation | None:
        if variant == BASE:
            return None
        return next(perturbation for perturbation in self.perturbations if perturbation.id == variant)


MASS = Perturbation(
    "mass_x0.5",
    "density of the concrete blocks of the walls x0.5 (half the thermal mass of the walls)",
    ENVELOPE,
    scale_material("CONCRETE_BLOCK:001", "density", 0.5),
)


def _insulation(factor: float) -> Change:
    return scale_material("FOAM_INSULATION:001", "thermal_conductivity", factor)


def _infiltration(factor: float) -> Change:
    return scale_space_parameter("ach", factor)


INSULATION = "conductivity of the wall insulation (FOAM_INSULATION, 0.04 W/(m.K))"
INFILTRATION = "infiltration (0.414 air changes per hour)"
BESTEST_ENVELOPE = (
    *sweep("insulation", INSULATION, ENVELOPE, (1.5,), _insulation),
    *sweep("infiltration", INFILTRATION, ENVELOPE, (2.0,), _infiltration),
    MASS,
)
# the sun and the gains: what drives the free-floating zones, and where FastModel keeps a little too much heat
BESTEST_SOLAR = (
    Perturbation(
        "glazing_tau_x0.75",
        "solar transmittance of the panes x0.75 (0.834 to 0.626: each pane absorbs 0.30 instead of 0.09)",
        ENVELOPE,
        scale_glass("CLEAR_GLASS:001", "solar_transmittance", 0.75),
    ),
    Perturbation(
        "absorptance_0.9",
        "solar absorptance of the outer surfaces of the walls and the roof 0.9 instead of 0.6",
        ENVELOPE,
        set_material(("WOOD_SIDING:001", "ROOF_DECK:001"), "shortwave_emissivity", 0.9),
    ),
    Perturbation("gains_x2", "internal gains x2 (200 to 400 W)", USE, scale_gains(2.0)),
)
BESTEST_940 = (
    *sweep("insulation", INSULATION, ENVELOPE, (0.5, 0.75, 1.5, 2.0), _insulation),
    *sweep("infiltration", INFILTRATION, ENVELOPE, (0.5, 1.5, 2.0, 3.0), _infiltration),
    MASS,
    Perturbation(
        "gap_k_x0.5",
        "conductivity of the gas of the glazing gap x0.5 (U-value of the glazing 2.9 to 2.6 W/(m2.K))",
        ENVELOPE,
        scale_gas("AIR:001", "thermal_conductivity", 0.5),
    ),
    *BESTEST_SOLAR,
    Perturbation(
        "setpoint_+2K",
        "heating set points +2 K (12 degC at night, 22 degC by day)",
        SYSTEM,
        ideal_setpoint_offset("heating_setpoint_schedule", 2.0),
    ),
)
SUNSPACE = ("SUNSPACE:001",)
BESTEST_960 = (
    Perturbation(
        "glazing_tau_x0.75",
        "solar transmittance of the panes of the sun-space x0.75 (0.834 to 0.626)",
        ENVELOPE,
        scale_glass("CLEAR_GLASS:001", "solar_transmittance", 0.75),
    ),
    Perturbation(
        "common_wall_k_x2",
        "conductivity of the concrete wall between the zone and the sun-space x2 (0.51 to 1.02 W/(m.K))",
        ENVELOPE,
        scale_material("CONCRETE_WALL:001", "thermal_conductivity", 2.0),
    ),
    Perturbation(
        "sunspace_mass_x0.5",
        "density of the concrete blocks of the sun-space walls x0.5",
        ENVELOPE,
        scale_material("CONCRETE_BLOCK:001", "density", 0.5),
    ),
    Perturbation(
        "sunspace_infiltration_x2",
        "infiltration of the sun-space x2 (0.414 to 0.828 air changes per hour)",
        ENVELOPE,
        scale_space_parameter("ach", 2.0, SUNSPACE),
    ),
)

RESIDENTIAL_PERTURBATIONS = (
    *sweep(
        "eps",
        "conductivity of the EPS of the facade insulation (0.032 W/(m.K))",
        ENVELOPE,
        (0.5, 1.5, 2.0),
        lambda factor: scale_material("EPS:001", "thermal_conductivity", factor),
    ),
    Perturbation(
        "n50_x2", "airtightness n50 x2 (3 to 6 1/h) in every room", ENVELOPE, scale_space_parameter("n50", 2.0)
    ),
    Perturbation(
        "heat_pump_2kW",
        "heat pump rated 2 kW instead of 3 kW",
        SYSTEM,
        system_parameter("boiler", "GENERATOR:001", "nominal_heating_power", 2000),
    ),
    Perturbation(
        "supply_+10K",
        "supply set point of the heat pump 50 degC instead of 40 degC",
        SYSTEM,
        system_parameter("boiler", "GENERATOR:001", "temperature_supply_setpoint_heat_pump", 323.15),
    ),
    Perturbation(
        "radiators_x0.7",
        "nominal power of every radiator x0.7",
        SYSTEM,
        scale_emission_parameter("radiator", "nominal_heating_power_positive_for_heating", 0.7),
    ),
    Perturbation(
        "tank_x3",
        "buffer tank 300 l instead of 100 l",
        SYSTEM,
        system_parameter("boiler", "GENERATOR:001", "tank_volume", 0.3),
    ),
    Perturbation(
        "heat_recovery_0.5",
        "effectiveness of the heat recovery 0.5 instead of 0.8",
        SYSTEM,
        system_parameter("air_handling_unit", "AHU:001", "heat_exchanger_effectiveness", 0.5),
    ),
    Perturbation(
        "setpoint_+1K",
        "set point of the living room and the kitchen 22 degC instead of 21 degC",
        SYSTEM,
        emission_control_parameter("temperature_heating_setpoint", 295.15, ("LIVING:001", "KITCHEN:001")),
    ),
    Perturbation(
        "ventilation_x1.5",
        "flow of the ventilation unit and of every duct x1.5 (150 to 225 m3/h)",
        SYSTEM,
        scale_ventilation(1.5),
    ),
    Perturbation(
        "radiator_n_1.33",
        "exponent of the EN 442 law of every radiator 1.33 instead of 1.24",
        SYSTEM,
        emission_parameter("radiator", "exponent_for_heat_transfer", 1.33),
    ),
)

CASES = {
    case.id: case
    for case in (
        StudyCase(
            id="950FF",
            path=BESTEST_CASES / "case_950FF.yaml",
            days=365,
            description="BESTEST 950FF: heavy single zone, free floating, night ventilation of 1700 m3/h from "
            "18:00 to 07:00, Denver",
            perturbations=(*BESTEST_ENVELOPE, *BESTEST_SOLAR),
            trace_windows=((7, 7), (190, 7)),
        ),
        StudyCase(
            id="940",
            path=BESTEST_CASES / "case_940.yaml",
            days=365,
            description="BESTEST 940: heavy single zone, ideal heating with a night set-back (10 degC from 23:00 to "
            "07:00, 20 degC by day) and cooling at 27 degC, Denver",
            perturbations=BESTEST_940,
            trace_windows=((7, 7), (190, 7)),
        ),
        StudyCase(
            id="960",
            path=BESTEST_CASES / "case_960.yaml",
            days=365,
            description="BESTEST 960: light zone heated and cooled (20/27 degC) behind an unconditioned heavy "
            "sun-space, Denver",
            perturbations=BESTEST_960,
            trace_windows=((7, 7), (190, 7)),
        ),
        StudyCase(
            id="residential_after",
            path=EXAMPLES / "residential_after_renovation.yaml",
            days=28,
            output_interval=300,
            description="Six-zone semi-detached house after renovation: air-to-water heat pump with a buffer, "
            "radiators with thermostatic valves, balanced ventilation with heat recovery, PV, January, Uccle",
            perturbations=RESIDENTIAL_PERTURBATIONS,
            trace_windows=((7, 7),),
        ),
    )
}
