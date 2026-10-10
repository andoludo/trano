"""The ASHRAE 140-2020 section 5.2 cases as trano building descriptions.

Every case is the 8 m x 6 m x 2.7 m single-zone box of the standard with the deltas that define it:
thermal mass, insulation, window orientation, glazing, shading, HVAC and its schedules, night
ventilation, and the sun-space of case 960. ``building_description`` turns a case into the YAML
dictionary trano reads; ``write_cases`` writes one file per case into ``cases/``.

Geometry and material data follow the standard as implemented in the Modelica Buildings library
(``Buildings.ThermalZones.Detailed.Validation.BESTEST``), which the reference results refer to.
"""

from __future__ import annotations

import math
from enum import Enum
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field

CASES_DIR = Path(__file__).parent.joinpath("cases")
WEATHER = 'Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos")'
PRESSURE_FROM_FILE = "Buildings.BoundaryConditions.Types.DataSource.File"
FRAME_FRACTION = 0.001  # the windows have no frame; a tiny one keeps the window model regular

LENGTH, WIDTH, HEIGHT = 8.0, 6.0, 2.7  # [m]
FLOOR_AREA = LENGTH * WIDTH  # [m2]
SUNSPACE_DEPTH = 2.0  # [m] case 960
WINDOW_AREA = 12.0  # [m2] in total, 2 m high: one south window, or one of 6 m2 on each of the east and west walls
WINDOW_HEIGHT = 2.0  # [m]
INFILTRATION = 0.414  # [1/h] 0.5 ACH at sea level corrected for the altitude of Denver
INTERNAL_GAIN = 200.0  # [W] continuous, 60 % radiant and 40 % convective
RADIANT_FRACTION = 0.6
SOLAR_ABSORPTANCE = 0.6
INFRARED_EMITTANCE = 0.9
REFERENCE_STATES = 18  # states per 0.2 m reference layer as in Buildings' BESTEST models; 3 under-resolves heavy walls
SOUTH, WEST, NORTH, EAST = 0.0, round(math.pi / 2, 6), round(math.pi, 6), round(-math.pi / 2, 6)  # [rad]
NIGHT_VENTILATION = 1700.0  # [m3/h] from 18:00 to 07:00, no fan heat
NIGHT_VENTILATION_MASS_FLOW = 1409.0 / 3600  # [kg/s] fan capacity at the altitude of the site, as Buildings' Case650
NIGHT_VENTILATION_HOURS = (18, 7)  # fan on from the first hour, off from the second
OVERHANG_DEPTH, OVERHANG_GAP = 1.0, 0.5  # [m] shading of cases 610, 630, 910, 930: depth, distance above the window
HEATING_SETPOINT, COOLING_SETPOINT = 20.0, 27.0  # [degC]
SETBACK = 10.0  # [degC] heating set point from 23:00 to 07:00 in cases 640 and 940
SINGLE_SETPOINT = (19.9, 20.1)  # [degC] cases 685, 695, 985, 995: 20 degC with a 0.2 K dead band
KELVIN = 273.15
COOLING_OFF = 100.0  # [degC] cooling set point outside the cooling hours
MAXIMUM_POWER = 1e6  # [W] capacity of the ideal system, as in the Buildings reference models


class Mass(str, Enum):
    light = "light"
    heavy = "heavy"


class Insulation(str, Enum):
    standard = "standard"
    high = "high"  # cases 680, 695, 980, 995


class Windows(str, Enum):
    south = "south"
    east_west = "east_west"


class Glazing(str, Enum):
    double_clear = "double_clear"
    double_low_e = "double_low_e"  # case 660: low-e outer pane, argon
    single_clear = "single_clear"  # case 670


class Shading(str, Enum):
    none = "none"
    overhang = "overhang"  # cases 610, 910: 1 m deep, 0.5 m above the window
    overhang_and_fins = "overhang_and_fins"  # cases 630, 930: plus 1 m deep side fins


def night_ventilation_schedule() -> str:
    """Mass flow table of the night ventilation fan: on from 18:00 to 07:00."""
    on, off = NIGHT_VENTILATION_HOURS
    flow = NIGHT_VENTILATION_MASS_FLOW
    rows = [(0, flow), (off * 3600, flow), (off * 3600, 0.0), (on * 3600, 0.0), (on * 3600, flow), (86400, flow)]
    return "[" + "; ".join(f"{time}, {value:.6g}" for time, value in rows) + "]"


def day_schedule(rows: list[tuple[int, float]]) -> str:
    """A Modelica table of (hour of the day, set point in degC) rows, in seconds and kelvin."""
    return "[" + "; ".join(f"{hour * 3600}, {value + KELVIN:g}" for hour, value in rows) + "]"


class Hvac(BaseModel):
    """Ideal heating and cooling of the zone air with dual set points."""

    heating_setpoint: float | None = HEATING_SETPOINT  # [degC], None: no heating
    cooling_setpoint: float | None = COOLING_SETPOINT  # [degC], None: no cooling
    heating_setback: float | None = None  # [degC] set point from 23:00 to 07:00, ramping up until 08:00
    cooling_hours: tuple[int, int] | None = None  # cooling only between these hours (cases 650, 950)

    @property
    def heating_schedule(self) -> str:
        setpoint = self.heating_setpoint if self.heating_setpoint is not None else 0.0
        if self.heating_setback is None:
            return day_schedule([(0, setpoint)])
        setback = self.heating_setback
        return day_schedule([(0, setback), (7, setback), (8, setpoint), (23, setpoint), (23, setback), (24, setback)])

    @property
    def cooling_schedule(self) -> str:
        setpoint = self.cooling_setpoint if self.cooling_setpoint is not None else COOLING_OFF
        if self.cooling_hours is None:
            return day_schedule([(0, setpoint)])
        start, end = self.cooling_hours
        off = COOLING_OFF
        return day_schedule([(0, off), (start, off), (start, setpoint), (end, setpoint), (end, off), (24, off)])

    @property
    def emission(self) -> dict[str, Any]:
        """The trano YAML emission element of the ideal system."""
        return {
            "ideal_heating_cooling": {
                "id": "HVAC:001",
                "parameters": {
                    "heating_setpoint_schedule": self.heating_schedule,
                    "cooling_setpoint_schedule": self.cooling_schedule,
                    "maximum_heating_power": 0.0 if self.heating_setpoint is None else MAXIMUM_POWER,
                    "maximum_cooling_power": 0.0 if self.cooling_setpoint is None else MAXIMUM_POWER,
                },
            }
        }


class Case(BaseModel):
    id: str
    description: str
    mass: Mass
    insulation: Insulation = Insulation.standard
    windows: Windows = Windows.south
    glazing: Glazing = Glazing.double_clear
    shading: Shading = Shading.none
    hvac: Hvac | None = None  # None: free-floating temperature
    night_ventilation: bool = False
    sunspace: bool = False  # case 960: the zone has no window, an unconditioned sun-space is attached south
    trace_days: list[tuple[int, int]] = Field(default_factory=lambda: [(2, 1)])  # days of the hourly output

    @property
    def free_float(self) -> bool:
        return self.hvac is None

    @property
    def features(self) -> set[str]:
        """What a library must support to run the case, beyond the plain envelope."""
        features: set[str] = set()
        if self.hvac is not None:
            features.add("hvac")
            if self.hvac.heating_setback is not None or self.hvac.cooling_hours is not None:
                features.add("setpoint_schedule")
        if self.night_ventilation:
            features.add("night_ventilation")
        if self.shading != Shading.none:
            features.add("shading")
        if self.sunspace:
            features.add("sunspace")
        return features


_SETBACK = Hvac(heating_setback=SETBACK)
_NIGHT_VENT = Hvac(heating_setpoint=None, cooling_hours=(7, 18))
_SINGLE = Hvac(heating_setpoint=SINGLE_SETPOINT[0], cooling_setpoint=SINGLE_SETPOINT[1])


def _case(
    id_: str, description: str, mass: Mass, hvac: Hvac | None = None, **changes: str | bool | list[tuple[int, int]]
) -> Case:
    return Case(id=id_, description=description, mass=mass, hvac=hvac, **changes)  # type: ignore[arg-type]


L, H = Mass.light, Mass.heavy
CASES: dict[str, Case] = {
    case.id: case
    for case in [
        _case("600", "Base case, low mass", L, Hvac()),
        _case("610", "600 with a south overhang", L, Hvac(), shading=Shading.overhang),
        _case("620", "600 with east and west windows", L, Hvac(), windows=Windows.east_west),
        _case(
            "630",
            "620 with overhangs and side fins",
            L,
            Hvac(),
            windows=Windows.east_west,
            shading=Shading.overhang_and_fins,
        ),
        _case("640", "600 with a night heating setback", L, _SETBACK),
        _case("650", "600 with night ventilation and no heating", L, _NIGHT_VENT, night_ventilation=True),
        _case("660", "600 with low-e argon double glazing", L, Hvac(), glazing=Glazing.double_low_e),
        _case("670", "600 with single pane windows", L, Hvac(), glazing=Glazing.single_clear),
        _case("680", "600 with more wall and roof insulation", L, Hvac(), insulation=Insulation.high),
        _case("685", "600 with a single 20 degC set point", L, _SINGLE),
        _case("695", "680 with a single 20 degC set point", L, _SINGLE, insulation=Insulation.high),
        _case("600FF", "600 free floating", L),
        _case("650FF", "650 free floating", L, night_ventilation=True, trace_days=[(7, 14)]),
        _case("680FF", "680 free floating", L, insulation=Insulation.high),
        _case("900", "Base case, high mass", H, Hvac()),
        _case("910", "900 with a south overhang", H, Hvac(), shading=Shading.overhang),
        _case("920", "900 with east and west windows", H, Hvac(), windows=Windows.east_west),
        _case(
            "930",
            "920 with overhangs and side fins",
            H,
            Hvac(),
            windows=Windows.east_west,
            shading=Shading.overhang_and_fins,
        ),
        _case("940", "900 with a night heating setback", H, _SETBACK),
        _case("950", "900 with night ventilation and no heating", H, _NIGHT_VENT, night_ventilation=True),
        _case("960", "Low mass zone with an unconditioned high mass sun-space", L, Hvac(), sunspace=True),
        _case("980", "900 with more wall and roof insulation", H, Hvac(), insulation=Insulation.high),
        _case("985", "900 with a single 20 degC set point", H, _SINGLE),
        _case("995", "980 with a single 20 degC set point", H, _SINGLE, insulation=Insulation.high),
        _case("900FF", "900 free floating", H),
        _case("950FF", "950 free floating", H, night_ventilation=True, trace_days=[(7, 14)]),
        _case("980FF", "980 free floating", H, insulation=Insulation.high),
    ]
}


# --------------------------------------------------------------------------- #
# Materials and constructions (layers listed outside first)
# --------------------------------------------------------------------------- #


def _material(id_: str, k: float, c: float, rho: float) -> dict[str, Any]:
    return {
        "id": id_,
        "thermal_conductivity": k,
        "specific_heat_capacity": c,
        "density": rho,
        "shortwave_emissivity": SOLAR_ABSORPTANCE,
        "longwave_emissivity": INFRARED_EMITTANCE,
        "number_of_states": REFERENCE_STATES,
    }


MATERIALS: dict[str, dict[str, Any]] = {
    material["id"]: material
    for material in [
        _material("WOOD_SIDING:001", 0.140, 900, 530),
        _material("FIBERGLASS:001", 0.040, 840, 12),
        _material("PLASTERBOARD:001", 0.160, 840, 950),
        _material("FOAM_INSULATION:001", 0.040, 1400, 10),
        _material("CONCRETE_BLOCK:001", 0.510, 1000, 1400),
        _material("ROOF_DECK:001", 0.140, 900, 530),
        # The standard gives the floor insulation no thermal mass; a negligible one (100 J/(m3.K), IDEAS'
        # own BESTEST data) keeps every library's layer discretization defined.
        _material("FLOOR_INSULATION:001", 0.040, 10, 10),
        _material("TIMBER_FLOOR:001", 0.140, 1200, 650),
        _material("CONCRETE_SLAB:001", 1.130, 1000, 1400),
        _material("CONCRETE_WALL:001", 0.510, 1000, 1400),  # common wall of case 960
    ]
}


def _construction(id_: str, *layers: tuple[str, float]) -> dict[str, Any]:
    return {"id": id_, "layers": [{"material": material, "thickness": thickness} for material, thickness in layers]}


SIDING, FIBERGLASS, BOARD = ("WOOD_SIDING:001", 0.009), "FIBERGLASS:001", ("PLASTERBOARD:001", 0.012)
FOAM, BLOCK, DECK = "FOAM_INSULATION:001", ("CONCRETE_BLOCK:001", 0.100), ("ROOF_DECK:001", 0.019)
CONSTRUCTIONS: dict[str, dict[str, Any]] = {
    construction["id"]: construction
    for construction in [
        _construction("LIGHT_WALL:001", SIDING, (FIBERGLASS, 0.066), BOARD),
        _construction("LIGHT_WALL_INSULATED:001", SIDING, (FOAM, 0.25), BOARD),
        _construction("HEAVY_WALL:001", SIDING, (FOAM, 0.0615), BLOCK),
        _construction("HEAVY_WALL_INSULATED:001", SIDING, (FOAM, 0.2452), BLOCK),
        _construction("ROOF:001", DECK, (FIBERGLASS, 0.1118), ("PLASTERBOARD:001", 0.010)),
        _construction("ROOF_INSULATED:001", DECK, (FIBERGLASS, 0.4), ("PLASTERBOARD:001", 0.010)),
        _construction("LIGHT_FLOOR:001", ("FLOOR_INSULATION:001", 1.003), ("TIMBER_FLOOR:001", 0.025)),
        _construction("HEAVY_FLOOR:001", ("FLOOR_INSULATION:001", 1.007), ("CONCRETE_SLAB:001", 0.080)),
        _construction("COMMON_WALL:001", ("CONCRETE_WALL:001", 0.2)),
    ]
}

GLASS_MATERIALS: dict[str, dict[str, Any]] = {
    "CLEAR_GLASS:001": {
        "id": "CLEAR_GLASS:001",
        "thermal_conductivity": 1.0,
        "density": 2500,
        "specific_heat_capacity": 840,
        "solar_transmittance": [0.834],
        "solar_reflectance_outside_facing": [0.075],
        "solar_reflectance_room_facing": [0.075],
        "infrared_transmissivity": 0,
        "infrared_absorptivity_outside_facing": 0.84,
        "infrared_absorptivity_room_facing": 0.84,
    },
    "LOW_E_GLASS:001": {  # coating on the gap side of the outer pane (case 660)
        "id": "LOW_E_GLASS:001",
        "thermal_conductivity": 1.0,
        "density": 2500,
        "specific_heat_capacity": 840,
        "solar_transmittance": [0.452],
        "solar_reflectance_outside_facing": [0.359],
        "solar_reflectance_room_facing": [0.397],
        "infrared_transmissivity": 0,
        "infrared_absorptivity_outside_facing": 0.84,
        "infrared_absorptivity_room_facing": 0.047,
    },
}
GASES: dict[str, dict[str, Any]] = {
    "AIR:001": {"id": "AIR:001", "thermal_conductivity": 0.025, "density": 1.2, "specific_heat_capacity": 1006},
    # Argon at 20 degC and 1 atm, where the glazing records of Buildings evaluate the gas properties.
    "ARGON:001": {"id": "ARGON:001", "thermal_conductivity": 0.0174, "density": 1.661, "specific_heat_capacity": 521.9},
}
GLAZINGS: dict[Glazing, dict[str, Any]] = {
    Glazing.double_clear: {
        "id": "DOUBLE_CLEAR:001",
        "u_value_frame": 1.4,
        "layers": [
            {"glass": "CLEAR_GLASS:001", "thickness": 0.003048},
            {"gas": "AIR:001", "thickness": 0.012},
            {"glass": "CLEAR_GLASS:001", "thickness": 0.003048},
        ],
    },
    Glazing.double_low_e: {
        "id": "DOUBLE_LOW_E:001",
        "u_value_frame": 1.4,
        "layers": [
            {"glass": "LOW_E_GLASS:001", "thickness": 0.003180},
            {"gas": "ARGON:001", "thickness": 0.012},
            {"glass": "CLEAR_GLASS:001", "thickness": 0.003048},
        ],
    },
    Glazing.single_clear: {
        "id": "SINGLE_CLEAR:001",
        "u_value_frame": 1.4,
        "layers": [{"glass": "CLEAR_GLASS:001", "thickness": 0.003048}],
    },
}


# Features of the standard a library cannot model with trano yet; its cases with them are skipped.
UNSUPPORTED_FEATURES: dict[str, frozenset[str]] = {
    "reduced_order": frozenset({"night_ventilation", "shading", "sunspace"}),
    "iso_13790": frozenset({"night_ventilation", "shading", "sunspace"}),
}


def check_support(case: Case, library: str) -> None:
    """Raise ``UnsupportedCaseError`` when the library cannot model a feature of the case."""
    unsupported = case.features & UNSUPPORTED_FEATURES.get(library, frozenset())
    if unsupported:
        raise UnsupportedCaseError(f"Case {case.id} needs {sorted(unsupported)}, not supported with {library} yet.")


class UnsupportedCaseError(NotImplementedError):
    """The case needs a feature the building description cannot express yet."""


def wall_construction(case: Case, mass: Mass | None = None) -> str:
    mass = mass or case.mass
    if mass == Mass.light:
        return "LIGHT_WALL_INSULATED:001" if case.insulation == Insulation.high else "LIGHT_WALL:001"
    return "HEAVY_WALL_INSULATED:001" if case.insulation == Insulation.high else "HEAVY_WALL:001"


def roof_construction(case: Case) -> str:
    return "ROOF_INSULATED:001" if case.insulation == Insulation.high else "ROOF:001"


def floor_construction(mass: Mass) -> str:
    return "LIGHT_FLOOR:001" if mass == Mass.light else "HEAVY_FLOOR:001"


def _wall(surface: float, azimuth: float, construction: str, tilt: str = "wall") -> dict[str, Any]:
    return {"surface": round(surface, 6), "azimuth": azimuth, "tilt": tilt, "construction": construction}


def _floor(surface: float, construction: str) -> dict[str, Any]:
    """A raised floor over outdoor air: the standard specifies no ground coupling."""
    return {"surface": surface, "construction": construction, "variant": "outdoor_air"}


def _window(azimuth: float, glazing: str, area: float = WINDOW_AREA, shading: Shading = Shading.none) -> dict[str, Any]:
    window: dict[str, Any] = {
        "surface": area,
        "azimuth": azimuth,
        "tilt": "wall",
        "construction": glazing,
        "width": area / WINDOW_HEIGHT,
        "height": WINDOW_HEIGHT,
        "frame_fraction": FRAME_FRACTION,
    }
    if shading == Shading.overhang:  # a 1 m deep overhang 0.5 m above the window, 0.5 m wider on each side
        window["overhang"] = {"depth": OVERHANG_DEPTH, "gap": OVERHANG_GAP, "width_left": 0.5, "width_right": 0.5}
    elif shading == Shading.overhang_and_fins:  # the overhang spans the window, 1 m deep fins at its edges
        window["overhang"] = {"depth": OVERHANG_DEPTH, "gap": OVERHANG_GAP, "width_left": 0.0, "width_right": 0.0}
        window["side_fins"] = {"depth": OVERHANG_DEPTH, "gap": 0.0, "height": OVERHANG_GAP}
    return window


def _occupancy(floor_area: float) -> dict[str, Any]:
    """The constant internal gain, written per floor area as the occupancy element expects."""
    radiant, convective = INTERNAL_GAIN * RADIANT_FRACTION, INTERNAL_GAIN * (1 - RADIANT_FRACTION)
    return {
        "parameters": {
            # Occupied all day: an entry at zero is read as "never occupied" by the schedule block.
            "occupancy": "{1, 86400}",
            "gain": f"[{radiant:g}/{floor_area:g}; {convective:g}/{floor_area:g}; 0]",
            "heat_gain_if_occupied": "1",
        }
    }


def _space(
    id_: str,
    floor_area: float,
    boundaries: dict[str, list[dict[str, Any]]],
    occupancy: dict[str, Any] | None,
) -> dict[str, Any]:
    space: dict[str, Any] = {
        "id": id_,
        "variant": "infiltration",
        "parameters": {
            "floor_area": floor_area,
            "average_room_height": HEIGHT,
            "ach": INFILTRATION,
            "linearize_emissive_power": "false",
        },
        "external_boundaries": boundaries,
    }
    if occupancy is not None:
        space["occupancy"] = occupancy
    return space


def _zone_boundaries(case: Case) -> dict[str, list[dict[str, Any]]]:
    wall = wall_construction(case)
    glazing = GLAZINGS[case.glazing]["id"]
    south_area, side_area = LENGTH * HEIGHT, WIDTH * HEIGHT
    walls = [
        _wall(south_area, NORTH, wall),
        _wall(side_area, EAST, wall),
        _wall(side_area, WEST, wall),
        _wall(FLOOR_AREA, SOUTH, roof_construction(case), tilt="ceiling"),
    ]
    windows: list[dict[str, Any]] = []
    if case.sunspace:
        pass  # the south side is the common wall with the sun-space
    elif case.windows == Windows.south:
        walls.insert(0, _wall(south_area, SOUTH, wall))
        windows.append(_window(SOUTH, glazing, shading=case.shading))
    else:
        walls.insert(0, _wall(south_area, SOUTH, wall))
        windows += [
            _window(EAST, glazing, WINDOW_AREA / 2, case.shading),
            _window(WEST, glazing, WINDOW_AREA / 2, case.shading),
        ]
    return {
        "external_walls": walls,
        "floor_on_grounds": [_floor(FLOOR_AREA, floor_construction(case.mass))],
        "windows": windows,
    }


def _sunspace_boundaries(case: Case) -> dict[str, list[dict[str, Any]]]:
    wall = wall_construction(case, Mass.heavy)
    area = LENGTH * SUNSPACE_DEPTH
    return {
        "external_walls": [
            _wall(LENGTH * HEIGHT, SOUTH, wall),
            _wall(SUNSPACE_DEPTH * HEIGHT, EAST, wall),
            _wall(SUNSPACE_DEPTH * HEIGHT, WEST, wall),
            _wall(area, SOUTH, roof_construction(case), tilt="ceiling"),
        ],
        "floor_on_grounds": [_floor(round(area, 6), floor_construction(Mass.heavy))],
        "windows": [_window(SOUTH, GLAZINGS[case.glazing]["id"])],
    }


def building_description(case: Case) -> dict[str, Any]:
    """The trano YAML content of a case."""
    unsupported = case.features & set()  # every feature of section 5.2 is supported
    if unsupported:
        raise UnsupportedCaseError(f"Case {case.id} needs {sorted(unsupported)}, not supported by trano yet.")
    zone = _space("ZONE:001", FLOOR_AREA, _zone_boundaries(case), _occupancy(FLOOR_AREA))
    if case.night_ventilation:
        zone["parameters"]["ventilation_schedule"] = night_ventilation_schedule()
    if case.hvac is not None:
        zone["emissions"] = [case.hvac.emission]
    spaces = [zone]
    internal_walls: list[dict[str, Any]] = []
    if case.sunspace:
        spaces.append(_space("SUNSPACE:001", LENGTH * SUNSPACE_DEPTH, _sunspace_boundaries(case), None))
        internal_walls.append(
            {
                "space_1": "ZONE:001",
                "space_2": "SUNSPACE:001",
                "construction": "COMMON_WALL:001",
                "surface": LENGTH * HEIGHT,
            }
        )
    used_constructions = sorted(
        {
            boundary["construction"]
            for space in spaces
            for kind in ("external_walls", "floor_on_grounds")
            for boundary in space["external_boundaries"][kind]
        }
        | {wall["construction"] for wall in internal_walls}
    )
    used_materials = sorted({layer["material"] for id_ in used_constructions for layer in CONSTRUCTIONS[id_]["layers"]})
    glazing = GLAZINGS[case.glazing]
    description: dict[str, Any] = {
        "material": [MATERIALS[id_] for id_ in used_materials],
        "gas": [GASES[layer["gas"]] for layer in glazing["layers"] if "gas" in layer],
        "glass_material": [
            GLASS_MATERIALS[id_] for id_ in sorted({layer["glass"] for layer in glazing["layers"] if "glass" in layer})
        ],
        "constructions": [CONSTRUCTIONS[id_] for id_ in used_constructions],
        "glazings": [glazing],
        "weather": {"parameters": {"path": WEATHER, "atmospheric_pressure_source": PRESSURE_FROM_FILE}},
        "spaces": spaces,
    }
    if internal_walls:
        description["internal_walls"] = internal_walls
    return description


def case_file(case_id: str, directory: Path = CASES_DIR) -> Path:
    return directory.joinpath(f"case_{case_id}.yaml")


def render_case(case: Case) -> str:
    header = (
        f"# ASHRAE 140-2020 case {case.id}: {case.description}. Generated by validation.bestest.cases, do not edit.\n"
    )
    return header + yaml.safe_dump(building_description(case), sort_keys=False, width=120)


def write_cases(directory: Path = CASES_DIR) -> list[Path]:
    """Write the YAML of every case the description can express; returns the files written."""
    directory.mkdir(parents=True, exist_ok=True)
    written = []
    for case in CASES.values():
        try:
            content = render_case(case)
        except UnsupportedCaseError:
            continue
        path = case_file(case.id, directory)
        path.write_text(content)
        written.append(path)
    return written
