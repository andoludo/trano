"""Write the residential reference cases of trano and print their sizing.

    uv run python examples/residential_cases.py

One description of a semi-detached Belgian house gives two YAML files, before and after a deep
renovation, that differ only by their envelope, airtightness and systems: geometry, zoning, use,
weather and radiators are shared. The design heat loss (EN 12831, simplified) sizes the radiators,
the boiler and the heat pump; the ground model of IDEAS (ISO 13370) gives Buildings the temperature
under the slab. See docs/validation/residential_reference_cases.md.
"""

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from trano.elements.construction import Gas, GasLayer, Glass, GlassLayer, GlassMaterial

OUTPUT = Path(__file__).parent
WEATHER = (
    'Modelica.Utilities.Files.loadResource("modelica://IDEAS/Resources/weatherdata/'
    'BEL_VLG_Uccle.064470_TMYx.2007-2021.mos")'
)
T_DESIGN = -8.0  # [degC] design outdoor temperature of Uccle (NBN EN 12831)
T_YEAR = 10.5  # [degC] annual mean outdoor temperature
T_SETBACK = 289.15  # [K] night setback of every room
T_ROOM_MEAN = 20.0  # [degC] heating-season mean of the ground-floor rooms (21 degC by day, 16 degC at night)
HEATING_MONTHS = (10, 11, 12, 1, 2, 3, 4)
RADIATOR_MARGIN = 1.2  # radiators of the old house: 1.2 times the room loss at 75/65/20 degC
RADIATOR_DT = 10.0  # [K] water temperature drop of the radiators at their rating
WATER_CP = 4186.0  # [J/(kg.K)]
SOUTH, WEST, NORTH, EAST = 0.0, 1.5708, 3.1416, -1.5708  # azimuths [rad]


@dataclass(frozen=True)
class Facade:
    azimuth: float  # [rad]
    gross_area: float  # [m2] windows included
    window_area: float  # [m2]


@dataclass(frozen=True)
class Room:
    name: str
    storey: int
    floor_area: float  # [m2]
    height: float  # [m]
    facades: tuple[Facade, ...]
    party_wall: float  # [m2] towards the heated neighbour
    roof: float  # [m2] ceiling under the cold attic
    slab_perimeter: float  # [m] exposed perimeter of the slab on ground (ground floor)
    occupancy: str  # on/off switching times [s]
    set_point: float  # [K]
    heating_schedule: str  # on/off switching times of the set point [s]
    ventilation: float  # [m3/h] supply and extract of the heat recovery ventilation (after renovation)

    @property
    def window_area(self) -> float:
        return sum(facade.window_area for facade in self.facades)

    @property
    def opaque_wall_area(self) -> float:
        return sum(facade.gross_area for facade in self.facades) - self.window_area


# Inside 7.4 m (east-west) x 8.4 m (north-south); ground floor 2.6 m high, first floor 2.5 m.
ROOMS = (
    Room("LIVING", 0, 40.0, 2.6, (Facade(SOUTH, 19.24, 6.5), Facade(WEST, 14.04, 1.5), Facade(NORTH, 7.8, 0.6)),
         21.84, 0.0, 15.8, "3600*{7, 8, 17, 23}", 294.15, "3600*{7, 23}", 40),
    Room("KITCHEN", 0, 15.0, 2.6, (Facade(NORTH, 11.44, 1.5), Facade(WEST, 8.84, 1.0)),
         0.0, 0.0, 7.8, "3600*{7, 8, 17, 20}", 294.15, "3600*{7, 23}", 25),
    Room("BEDROOM_1", 1, 18.5, 2.5, (Facade(SOUTH, 11.0, 2.2), Facade(WEST, 10.5, 0.0)),
         0.0, 18.5, 0.0, "3600*{0, 7, 22, 24}", 291.15, "3600*{6, 23}", 25),
    Room("BEDROOM_2", 1, 18.5, 2.5, (Facade(NORTH, 11.0, 2.0), Facade(WEST, 10.5, 0.8)),
         0.0, 18.5, 0.0, "3600*{0, 7, 21, 24}", 291.15, "3600*{6, 23}", 25),
    Room("BEDROOM_3", 1, 9.6, 2.5, (Facade(NORTH, 7.5, 1.2),),
         8.0, 9.6, 0.0, "3600*{0, 7, 21, 24}", 291.15, "3600*{6, 23}", 15),
    Room("BATHROOM", 1, 9.0, 2.5, (Facade(SOUTH, 7.5, 0.8),),
         7.5, 9.0, 0.0, "3600*{6, 8, 20, 22}", 295.15, "3600*{6, 9, 19, 22}", 20),
)  # fmt: skip
# (upper room, lower room, area) floors between the storeys; (room, room, area) walls between rooms
FLOORS = (("BEDROOM_1", "LIVING", 18.5), ("BATHROOM", "LIVING", 9.0), ("BEDROOM_3", "LIVING", 9.6),
          ("BEDROOM_2", "KITCHEN", 15.0), ("BEDROOM_2", "LIVING", 3.5))  # fmt: skip
WALLS = (("LIVING", "KITCHEN", 20.3), ("BEDROOM_1", "BEDROOM_2", 11.0), ("BEDROOM_1", "BATHROOM", 10.5),
         ("BATHROOM", "BEDROOM_3", 7.5), ("BEDROOM_2", "BEDROOM_3", 8.0))  # fmt: skip

MATERIALS = """\
material:
  - {id: BRICK_FACING:001, thermal_conductivity: 0.9, density: 1700, specific_heat_capacity: 840}
  - {id: BRICK_INNER:001, thermal_conductivity: 0.7, density: 1600, specific_heat_capacity: 840}
  # unventilated air layers as equivalent conductivities (EN ISO 6946): 5 cm cavity R = 0.18 m2K/W,
  # 10 cm between the ceiling joists R = 0.16 m2K/W
  - {id: CAVITY_AIR_5CM:001, thermal_conductivity: 0.278, density: 1.2, specific_heat_capacity: 1006}
  - {id: JOIST_AIR_10CM:001, thermal_conductivity: 0.625, density: 1.2, specific_heat_capacity: 1006}
  - {id: PLASTER:001, thermal_conductivity: 0.5, density: 1300, specific_heat_capacity: 1000}
  - {id: GYPSUM_BOARD:001, thermal_conductivity: 0.25, density: 900, specific_heat_capacity: 1000}
  - {id: TIMBER:001, thermal_conductivity: 0.13, density: 500, specific_heat_capacity: 1600}
  - {id: TILE:001, thermal_conductivity: 1.3, density: 2300, specific_heat_capacity: 840}
  - {id: SCREED:001, thermal_conductivity: 1.4, density: 2000, specific_heat_capacity: 1000}
  - {id: CONCRETE:001, thermal_conductivity: 1.7, density: 2300, specific_heat_capacity: 1000}
{extra}
gas:
  - {id: AIR:001, thermal_conductivity: 0.0256, density: 1.23, specific_heat_capacity: 1006,
     longwave_emissivity: 0.0, shortwave_emissivity: 0.0}
  - {id: ARGON:001, thermal_conductivity: 0.0168, density: 1.70, specific_heat_capacity: 519,
     longwave_emissivity: 0.0, shortwave_emissivity: 0.0}
glass_material:
  # 4 mm float glass; the low-e panes carry a coating (emissivity 0.04) on the face given
  - id: GLASS:001
    thermal_conductivity: 1.0
    density: 2500
    specific_heat_capacity: 840
    solar_transmittance: [0.83]
    solar_reflectance_outside_facing: [0.075]
    solar_reflectance_room_facing: [0.075]
    infrared_transmissivity: 0.0
    infrared_absorptivity_outside_facing: 0.84
    infrared_absorptivity_room_facing: 0.84
  - id: GLASS_LOWE_ROOM_SIDE:001
    thermal_conductivity: 1.0
    density: 2500
    specific_heat_capacity: 840
    solar_transmittance: [0.62]
    solar_reflectance_outside_facing: [0.2]
    solar_reflectance_room_facing: [0.2]
    infrared_transmissivity: 0.0
    infrared_absorptivity_outside_facing: 0.84
    infrared_absorptivity_room_facing: 0.04
  - id: GLASS_LOWE_OUTSIDE_SIDE:001
    thermal_conductivity: 1.0
    density: 2500
    specific_heat_capacity: 840
    solar_transmittance: [0.62]
    solar_reflectance_outside_facing: [0.2]
    solar_reflectance_room_facing: [0.2]
    infrared_transmissivity: 0.0
    infrared_absorptivity_outside_facing: 0.04
    infrared_absorptivity_room_facing: 0.84
"""

SHARED_CONSTRUCTIONS = """\
  # half of the 19 cm brick party wall: its mid-plane is adiabatic (the neighbour is heated alike)
  - id: PARTY_WALL:001
    layers:
      - {material: BRICK_INNER:001, thickness: 0.095}
      - {material: PLASTER:001, thickness: 0.015}
  - id: INTERNAL_WALL:001
    layers:
      - {material: PLASTER:001, thickness: 0.01}
      - {material: BRICK_INNER:001, thickness: 0.09}
      - {material: PLASTER:001, thickness: 0.01}
  # timber floor between the storeys: boards on joists, plasterboard ceiling
  - id: INTERMEDIATE_FLOOR:001
    layers:
      - {material: TIMBER:001, thickness: 0.022}
      - {material: JOIST_AIR_10CM:001, thickness: 0.1}
      - {material: GYPSUM_BOARD:001, thickness: 0.015}
"""


@dataclass(frozen=True)
class Case:
    name: str
    title: str
    extra_materials: str
    constructions: str
    glazing: str
    frame_fraction: float
    n50: float  # [1/h]
    thermal_bridges: float  # [W/(m2.K)] added to the U-values in the sizing (not simulated)
    summary: str

    @property
    def materials(self) -> str:
        return MATERIALS.replace("{extra}", self.extra_materials)

    def layers(self, construction_id: str) -> list[dict[str, Any]]:
        constructions = yaml.safe_load(self.constructions + SHARED_CONSTRUCTIONS)
        return next(c["layers"] for c in constructions if c["id"] == construction_id)

    def resistance(self, construction_id: str) -> float:
        """Conduction resistance of a construction [m2.K/W]."""
        conductivity = {m["id"]: m["thermal_conductivity"] for m in yaml.safe_load(self.materials)["material"]}
        return float(
            sum(layer["thickness"] / conductivity[layer["material"]] for layer in self.layers(construction_id))
        )

    def window(self) -> tuple[float, float, float]:
        """Uw, Ug (EN 673, as trano computes it for IDEAS) and g of the windows."""
        data = yaml.safe_load(self.materials)
        glasses = {g["id"]: GlassMaterial(name=g["id"], **{k: v for k, v in g.items() if k != "id"})
                   for g in data["glass_material"]}  # fmt: skip
        gases = {g["id"]: Gas(name=g["id"], **{k: v for k, v in g.items() if k != "id"}) for g in data["gas"]}
        glazing = yaml.safe_load(self.glazing)[0]
        layers: list[GlassLayer | GasLayer] = [
            GlassLayer(material=glasses[layer["glass"]], thickness=layer["thickness"])
            if "glass" in layer
            else GasLayer(material=gases[layer["gas"]], thickness=layer["thickness"])
            for layer in glazing["layers"]
        ]
        properties = Glass(name="window", layers=layers, u_value_frame=glazing["u_value_frame"]).properties
        u_window = (1 - self.frame_fraction) * properties.u_value + self.frame_fraction * glazing["u_value_frame"]
        return u_window, properties.u_value, properties.g_value


BEFORE = Case(
    name="before",
    title="Semi-detached house built before 1970, before renovation",
    extra_materials="",
    constructions="""\
  # cavity wall without insulation, U = 1.47 W/m2K
  - id: EXTERNAL_WALL:001
    layers:
      - {material: BRICK_FACING:001, thickness: 0.09}
      - {material: CAVITY_AIR_5CM:001, thickness: 0.05}
      - {material: BRICK_INNER:001, thickness: 0.14}
      - {material: PLASTER:001, thickness: 0.015}
  # ceiling under the cold attic without insulation, U = 1.70 W/m2K
  - id: ROOF:001
    layers:
      - {material: TIMBER:001, thickness: 0.022}
      - {material: JOIST_AIR_10CM:001, thickness: 0.1}
      - {material: GYPSUM_BOARD:001, thickness: 0.015}
  # slab on ground without insulation
  - id: GROUND_FLOOR:001
    layers:
      - {material: CONCRETE:001, thickness: 0.12}
      - {material: SCREED:001, thickness: 0.06}
      - {material: TILE:001, thickness: 0.01}
""",
    glazing="""\
  # old double glazing 4-12-4 air in wooden frames (Uw about 2.7 W/m2K)
  - id: WINDOW:001
    u_value_frame: 2.2
    layers:
      - {glass: GLASS:001, thickness: 0.004}
      - {gas: AIR:001, thickness: 0.012}
      - {glass: GLASS:001, thickness: 0.004}
""",
    frame_fraction=0.25,
    n50=12.0,
    thermal_bridges=0.10,
    summary="Gas boiler, radiators, natural ventilation by infiltration (n50 = 12 1/h).",
)
AFTER = Case(
    name="after",
    title="The same house after a deep renovation",
    extra_materials="""\
  - {id: EPS:001, thermal_conductivity: 0.032, density: 20, specific_heat_capacity: 1450}
  - {id: MINERAL_WOOL:001, thermal_conductivity: 0.035, density: 30, specific_heat_capacity: 1030}
  - {id: PUR:001, thermal_conductivity: 0.023, density: 35, specific_heat_capacity: 1400}""",
    constructions="""\
  # the cavity wall with 12 cm of external insulation (ETICS), U = 0.23 W/m2K
  - id: EXTERNAL_WALL:001
    layers:
      - {material: EPS:001, thickness: 0.12}
      - {material: BRICK_FACING:001, thickness: 0.09}
      - {material: CAVITY_AIR_5CM:001, thickness: 0.05}
      - {material: BRICK_INNER:001, thickness: 0.14}
      - {material: PLASTER:001, thickness: 0.015}
  # 22 cm of mineral wool on the attic floor, U = 0.15 W/m2K
  - id: ROOF:001
    layers:
      - {material: MINERAL_WOOL:001, thickness: 0.22}
      - {material: TIMBER:001, thickness: 0.022}
      - {material: JOIST_AIR_10CM:001, thickness: 0.1}
      - {material: GYPSUM_BOARD:001, thickness: 0.015}
  # 8 cm of PUR under a new screed
  - id: GROUND_FLOOR:001
    layers:
      - {material: CONCRETE:001, thickness: 0.12}
      - {material: PUR:001, thickness: 0.08}
      - {material: SCREED:001, thickness: 0.06}
      - {material: TILE:001, thickness: 0.01}
""",
    glazing="""\
  # triple glazing 4-12-4-12-4 argon with two low-e coatings in insulated frames (Uw about 0.8 W/m2K)
  - id: WINDOW:001
    u_value_frame: 1.1
    layers:
      - {glass: GLASS_LOWE_ROOM_SIDE:001, thickness: 0.004}
      - {gas: ARGON:001, thickness: 0.012}
      - {glass: GLASS:001, thickness: 0.004}
      - {gas: ARGON:001, thickness: 0.012}
      - {glass: GLASS_LOWE_OUTSIDE_SIDE:001, thickness: 0.004}
""",
    frame_fraction=0.25,
    n50=3.0,
    thermal_bridges=0.05,
    summary="Air-water heat pump on the radiators of the old house, balanced ventilation with heat recovery, "
    "PV, n50 = 3 1/h.",
)
HEAT_RECOVERY = 0.8

BOILER = """\
# non-condensing gas boiler (efficiency 0.85 on the lower heating value), 75 degC supply
parameters:
  nominal_heating_power: {power}
  supply_temperature_setpoint: 348.15
  coefficients_for_efficiency_curve: "{{0.85}}"
variant: without_storage"""
HEAT_PUMP = """\
# air-water heat pump rated {kw} kW at -8 degC and 40 degC supply (heating curve down to 30 degC at
# 16 degC outdoor), 100 l buffer
parameters:
  nominal_heating_power: {power}
  temperature_supply_setpoint_heat_pump: 313.15
  use_storage_tank: true
  tank_volume: 0.1
  height_of_tank_without_insulation: 0.8
variant: air_water_heat_pump"""
PV = """\
solar:
  # 21 m2 of 20 % modules (4.2 kWp) on the south roof slope at 35 degrees
  - photovoltaic:
      id: PV:001
      parameters:
        area: 21.0
        efficiency: 0.2
        azimuth: 0.0
        tilt: 35.0"""


# --------------------------------------------------------------------------------------------------
# Ground: IDEAS.Buildings.Components.SlabOnGround (IDEAS 3.0, ISO 13370) prescribes a monthly heat flow
# under 1 m of ground; Buildings holds the outer face of the slab at a fixed temperature.
def ideas_ground_heat_flow(case: Case, area: float, perimeter: float, month: float) -> float:
    """Heat flow to the ground of IDEAS' SlabOnGround [W] with TiAvg = T_ROOM_MEAN (month 0 = 1 January)."""
    k, rho, c = 2.0, 1600.0, 1250.0  # IDEAS.Buildings.Data.Materials.Ground
    t_e_avg, dt_i, dt_e = 10.8, 2.0, 4.0  # IDEAS defaults
    u_floor = 1 / (1 / 6 + case.resistance("GROUND_FLOOR:001"))
    b = area / (0.5 * perimeter)
    dt = float(sum(layer["thickness"] for layer in case.layers("GROUND_FLOOR:001"))) + k / u_floor
    # equivalent U of the slab (EN ISO 13370), for a wide or a narrow slab
    u_equivalent = 2 * k / (math.pi * b + dt) * math.log(math.pi * b / dt + 1) if dt < b else k / (0.457 * b + dt)
    delta = math.sqrt(3.15e7 * k / 3.14 / rho / c)
    alfa = 1.5 - 12 / (2 * 3.14) * math.atan(dt / (dt + delta))
    beta = 1.5 - 0.42 * math.log(delta / (dt + 1))
    l_pi = area * k / dt * math.sqrt(1 / ((1 + delta / dt) ** 2 + 1))
    l_pe = 0.37 * perimeter * k * math.log(delta / dt + 1)
    return (
        u_equivalent * area * (T_ROOM_MEAN - t_e_avg)
        - l_pi * dt_i * math.cos(2 * 3.1415 / 12 * (month - 1 + alfa))
        + l_pe * dt_e * math.cos(2 * 3.1415 / 12 * (month - 1 - beta))
    )


def heating_season_ground_heat_flow(case: Case, room: Room) -> float:
    """Mean heat flow to the ground of IDEAS over the heating season [W]."""
    flows = [
        ideas_ground_heat_flow(case, room.floor_area, room.slab_perimeter, month - 0.5) for month in HEATING_MONTHS
    ]
    return sum(flows) / len(flows)


def buildings_ground_temperature(case: Case, room: Room) -> float:
    """Temperature under the Buildings slab [K] giving the heating-season ground heat flow of IDEAS."""
    resistance = 0.17 + case.resistance("GROUND_FLOOR:001")
    return T_ROOM_MEAN + 273.15 - heating_season_ground_heat_flow(case, room) * resistance / room.floor_area


# --------------------------------------------------------------------------------------------------
# Design heat loss (EN 12831, simplified)
def design_heat_loss(case: Case) -> dict[str, float]:
    """Design heat loss of every room [W] at T_DESIGN."""
    u_wall = 1 / (0.13 + case.resistance("EXTERNAL_WALL:001") + 0.04)
    u_roof = 1 / (0.10 + case.resistance("ROOF:001") + 0.10)  # Rse 0.10 under a cold attic
    u_window = case.window()[0]
    losses = {}
    for room in ROOMS:
        t_room = room.set_point - 273.15
        transmission = (
            (u_wall + case.thermal_bridges) * room.opaque_wall_area
            + (u_window + case.thermal_bridges) * room.window_area
            + (u_roof + case.thermal_bridges) * room.roof
        )
        volume = room.floor_area * room.height
        if case is BEFORE:  # natural ventilation: at least 0.5 1/h (EN 12831)
            ventilation = 0.34 * max(case.n50 / 20, 0.5) * volume
        else:  # infiltration and the supply air after the heat recovery
            ventilation = 0.34 * case.n50 / 20 * volume + 0.34 * room.ventilation * (1 - HEAT_RECOVERY)
        # ground: the heating-season heat flow of the ISO 13370 model of IDEAS, scaled to the design conditions
        ground = (
            heating_season_ground_heat_flow(case, room) * (t_room - T_YEAR) / (T_ROOM_MEAN - T_YEAR) * 1.45
            if room.storey == 0
            else 0.0
        )
        losses[room.name] = (transmission + ventilation) * (t_room - T_DESIGN) + ground
    return losses


def radiator_powers() -> dict[str, float]:
    """Radiators of the old house at 75/65/20 degC, kept after the renovation [W]."""
    return {
        name: max(400.0, math.ceil(RADIATOR_MARGIN * loss / 100) * 100)
        for name, loss in design_heat_loss(BEFORE).items()
    }


def kept_radiators_supply_temperature() -> int:
    """Lowest supply temperature [degC] at which the kept radiators cover every room after renovation."""
    radiators, losses = radiator_powers(), design_heat_loss(AFTER)
    worst = max(losses[name] / radiators[name] for name in radiators)
    rating = (75 - 65) / math.log((75 - 20) / (65 - 20))
    for supply in range(30, 76):
        log_mean = 7 / math.log((supply - 20) / (supply - 27))
        if (log_mean / rating) ** 1.3 >= worst:
            return supply
    raise ValueError("the radiators cannot cover the loss")


# --------------------------------------------------------------------------------------------------
# YAML
def _room_yaml(case: Case, room: Room, radiator: float, ventilated: bool) -> list[str]:
    lines = [
        f"  - id: {room.name}:001",
        "    variant: infiltration",
        "    occupancy:",
        "      parameters:",
        f"        occupancy: {room.occupancy}",
        "    parameters:",
        f"      floor_area: {room.floor_area}",
        f"      average_room_height: {room.height}",
        f"      n50: {case.n50}",
        "    external_boundaries:",
        "      external_walls:",
        *(
            f"        - {{surface: {facade.gross_area}, azimuth: {facade.azimuth}, tilt: wall, "
            "construction: EXTERNAL_WALL:001}"
            for facade in room.facades
        ),
    ]
    if room.roof:
        lines.append(f"        - {{surface: {room.roof}, azimuth: 0, tilt: ceiling, construction: ROOF:001}}")
    windows = [facade for facade in room.facades if facade.window_area]
    if windows:
        lines.append("      windows:")
        lines += [
            f"        - {{surface: {facade.window_area}, azimuth: {facade.azimuth}, tilt: wall, "
            f"construction: WINDOW:001, frame_fraction: {case.frame_fraction}}}"
            for facade in windows
        ]
    if room.party_wall:
        lines += [
            "      adiabatic_walls:",
            f"        - {{surface: {room.party_wall}, azimuth: {EAST}, tilt: wall, construction: PARTY_WALL:001}}",
        ]
    if room.storey == 0:
        lines += [
            "      floor_on_grounds:",
            f"        - surface: {room.floor_area}",
            "          construction: GROUND_FLOOR:001",
            "          # library-specific: IDEAS (ISO 13370 ground model) takes the exposed perimeter and the",
            "          # mean room temperature, Buildings the temperature under the slab that gives the heat flow",
            "          # of that ground model averaged over the heating season (October to April)",
            f"          perimeter: {room.slab_perimeter}",
            f"          average_indoor_temperature: {T_ROOM_MEAN + 273.15}",
            f"          ground_temperature: {buildings_ground_temperature(case, room):.2f}",
        ]
    lines += [
        "    emissions:",
        "      - radiator:",
        f"          id: RADIATOR_{room.name}:001",
        "          parameters:",
        f"            nominal_heating_power_positive_for_heating: {radiator:.0f}",
        "            water_inlet_temperature_at_nominal_condition: 348.15",
        "            water_outlet_temperature_at_nominal_condition: 338.15",
        "            air_temperature_at_nominal_condition: 293.15",
        "      - valve:",
        f"          id: VALVE_{room.name}:001",
        "          parameters:",
        f"            m_flow_nominal: {radiator / WATER_CP / RADIATOR_DT:.3f}",
        "          control:",
        "            emission_control:",
        f"              id: EMISSION_CONTROL_{room.name}:001",
        "              variant: PController",
        "              parameters:",
        f"                schedule: {room.heating_schedule}",
        f"                temperature_heating_setpoint: {room.set_point}",
        f"                temperature_heating_setback: {T_SETBACK}",
    ]
    if ventilated:
        flow = room.ventilation * 1.2 / 3600
        for kind, side in (("ventilation_inlets", "SUPPLY"), ("ventilation_outlets", "EXTRACT")):
            lines += [
                f"    {kind}:",
                "      - duct:",
                f"          id: DUCT_{side}_{room.name}:001",
                "          parameters:",
                f"            nominal_mass_flow_rate: {flow:.5f}",
                "            dp_nominal: 50",
            ]
    return lines


def _systems_yaml(generator: str, pump_flow: float, ventilated: bool) -> list[str]:
    lines = [
        "  - boiler:",
        "      id: GENERATOR:001",
        "      control:",
        "        boiler_control:",
        "          id: GENERATOR_CONTROL:001",
        "          variant: simple",
        *("      " + line for line in generator.splitlines()),
        "      inlets:",
        "        - SPLIT_VALVE:001",
        "  - pump:",
        "      id: PUMP:001",
        "      parameters:",
        f"        m_flow_nominal: {pump_flow:.3f}",
        "      control:",
        "        collector_control:",
        "      inlets:",
        "        - GENERATOR:001",
        "  - three_way_valve:",
        "      id: THREE_WAY_VALVE:001",
        "      parameters:",
        f"        m_flow_nominal: {pump_flow:.3f}",
        "      control:",
        "        three_way_valve_control:",
        "      inlets:",
        "        - SPLIT_VALVE:001",
        "        - PUMP:001",
        "      outlets:",
        "        - TEMPERATURE_SENSOR:001",
        "  - temperature_sensor:",
        "      id: TEMPERATURE_SENSOR:001",
        "      outlets:",
        *(f"        - RADIATOR_{room.name}:001" for room in ROOMS),
        "  - split_valve:",
        "      id: SPLIT_VALVE:001",
        "      inlets:",
        *(f"        - VALVE_{room.name}:001" for room in ROOMS),
        "      outlets:",
        "        - GENERATOR:001",
    ]
    if ventilated:
        total = sum(room.ventilation for room in ROOMS)
        lines += [
            f"  # balanced ventilation with heat recovery (system D), {total:.0f} m3/h; the fans hold 150 Pa:",
            "  # 100 Pa through the heat exchanger and 50 Pa through each duct at their nominal flow",
            "  - air_handling_unit:",
            "      id: AHU:001",
            "      variant: system_d",
            "      parameters:",
            f"        m_flow_nominal: {total * 1.2 / 3600:.5f}",
            "        dp_nominal: 100",
            "        supply_dp_nominal: 150",
            "        return_dp_nominal: 150",
            f"        heat_exchanger_effectiveness: {HEAT_RECOVERY}",
            "      inlets:",
            *(f"        - DUCT_EXTRACT_{room.name}:001" for room in ROOMS),
            "      outlets:",
            *(f"        - DUCT_SUPPLY_{room.name}:001" for room in ROOMS),
        ]
    return lines


def case_yaml(case: Case, generator: str, solar: str | None) -> str:
    radiators = radiator_powers()
    loss = sum(design_heat_loss(case).values()) / 1000
    header = f"""\
# {case.title}
#
# Reference case of trano: one YAML generates the model of either library,
#   trano create-model examples/residential_{case.name}_renovation.yaml IDEAS      (or Buildings)
# Semi-detached two-storey house in Uccle (Belgium), 110 m2 heated in six zones: living (with the hall)
# and kitchen on the ground floor, three bedrooms and the bathroom upstairs. Street facade north,
# garden facade south, gable west, party wall east (adiabatic: the neighbour is heated alike).
# Four occupants; 21 degC in the living spaces, 18 degC in the bedrooms, 22 degC in the bathroom,
# 16 degC setback.
#
# Design heat loss (EN 12831 at -8 degC): {loss:.1f} kW. {case.summary}
#
# Library-specific parameters (all else is shared): the floors on ground give IDEAS the exposed
# perimeter of the slab and the mean room temperature (ISO 13370 ground model) and Buildings the
# temperature under the slab that gives the same heat flow.
# Sizing, assumptions and results: docs/validation/residential_reference_cases.md (written by
# examples/residential_cases.py).
"""
    pump_flow = 1.1 * sum(radiators.values()) / WATER_CP / RADIATOR_DT
    ventilated = case is AFTER
    lines = [header, case.materials.rstrip("\n"), "", "constructions:"]
    lines += [case.constructions.rstrip("\n"), SHARED_CONSTRUCTIONS.rstrip("\n"), ""]
    lines += ["glazings:", case.glazing.rstrip("\n"), ""]
    lines += ["weather:", "  parameters:", f"    path: '{WEATHER}'", "", "spaces:"]
    for room in ROOMS:
        lines += _room_yaml(case, room, radiators[room.name], ventilated)
    lines += ["", "internal_walls:"]
    lines += [
        f"  - {{space_1: {upper}:001, space_2: {lower}:001, construction: INTERMEDIATE_FLOOR:001, "
        f"surface: {area}, space_1_tilt: floor, space_2_tilt: ceiling}}"
        for upper, lower, area in FLOORS
    ]
    lines += [
        f"  - {{space_1: {first}:001, space_2: {second}:001, construction: INTERNAL_WALL:001, surface: {area}}}"
        for first, second, area in WALLS
    ]
    lines += ["", "systems:", *_systems_yaml(generator, pump_flow, ventilated)]
    if solar:
        lines += ["", solar]
    return "\n".join(lines) + "\n"


def boiler_power() -> int:
    """Gas boiler of the old house: 1.4 times the design loss (reheat after the setback), in steps of 1 kW."""
    return math.ceil(1.4 * sum(design_heat_loss(BEFORE).values()) / 1000) * 1000


def heat_pump_power() -> int:
    """Heat pump: 1.25 times the design loss after renovation, in steps of 0.5 kW."""
    return math.ceil(1.25 * sum(design_heat_loss(AFTER).values()) / 500) * 500


def print_sizing() -> None:
    for case in (BEFORE, AFTER):
        u_window, u_glass, g = case.window()
        print(
            f"[{case.name}] U wall {1 / (0.17 + case.resistance('EXTERNAL_WALL:001')):.2f}, "
            f"roof {1 / (0.2 + case.resistance('ROOF:001')):.2f}, glazing Ug {u_glass:.2f} g {g:.2f}, "
            f"window Uw {u_window:.2f}, n50 {case.n50}"
        )
        for room in ROOMS:
            if room.storey == 0:
                print(
                    f"  ground {room.name}: IDEAS heating-season flow "
                    f"{heating_season_ground_heat_flow(case, room):.0f} W, Buildings ground temperature "
                    f"{buildings_ground_temperature(case, room) - 273.15:.2f} degC"
                )
    before, after, radiators = design_heat_loss(BEFORE), design_heat_loss(AFTER), radiator_powers()
    print(f"{'room':10s} {'before W':>9s} {'after W':>8s} {'radiator W':>11s}")
    for name, radiator in radiators.items():
        print(f"{name:10s} {before[name]:9.0f} {after[name]:8.0f} {radiator:11.0f}")
    print(
        f"total: before {sum(before.values()) / 1000:.2f} kW (boiler {boiler_power() / 1000:.0f} kW), "
        f"after {sum(after.values()) / 1000:.2f} kW (heat pump {heat_pump_power() / 1000:.1f} kW), "
        f"radiators {sum(radiators.values()) / 1000:.1f} kW at 75/65/20 degC; the kept radiators cover the "
        f"loss after renovation from {kept_radiators_supply_temperature()} degC supply"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=OUTPUT, help="Directory of the YAML files")
    output = parser.parse_args().output
    print_sizing()
    heat_pump = heat_pump_power()
    cases = (
        (BEFORE, BOILER.format(power=boiler_power()), None),
        (AFTER, HEAT_PUMP.format(power=heat_pump, kw=heat_pump / 1000), PV),
    )
    for case, generator, solar in cases:
        path = output.joinpath(f"residential_{case.name}_renovation.yaml")
        path.write_text(case_yaml(case, generator, solar))
        print(f"written {path}")


if __name__ == "__main__":
    main()
