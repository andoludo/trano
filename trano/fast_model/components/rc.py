"""Connected version of the RC zone models of the ``FastModel`` library.

Each RC structure of :mod:`trano.mpc.parameters` is assembled from ``FastModel.Components``
primitives and reproduces the equations of the flat ``building_mpc`` exactly (tested): the
capacitor of state ``<zone>_<node>`` of the flat model is the instance ``<zone>_<node>``, so its
CasADi state is ``<zone>_<node>.T``.

* ``R1C1``: one capacity; window and opaque solar gains, internal gains and heating on the air node.
* ``R3C2``: air and envelope; window solar on the air, opaque solar on the envelope.
* ``R4C3``: ``R3C2`` plus the emitter node, which receives the heating.
* ``ISO13790``: air and mass nodes, the massless surface node solved in closed form
  (``Thermal.StarNode3``); radiant gains split between surface and mass as in ISO 13790.

The energy systems of the flat model are not connected yet: buildings with systems keep the flat model.
"""

from trano.mpc.building import RCBuilding, RCZone
from trano.fast_model.components.model import ConnectedModel, Instance, TopInput
from trano.mpc.parameters import ISO13790Parameters, R1C1Parameters, R3C2Parameters, R4C3Parameters

OUTDOOR = "outdoor"
_SOLAR_NAMES = {"gA": "solWin", "aE": "solOpa"}
GROUND = "ground"


def state_instance(zone: RCZone, node: str) -> str:
    """Instance holding the state ``node`` (``Ti``, ``Te``, ``Th``, ``Tm``) of a zone."""
    return f"{zone.prefix}{node}"


def indoor_temperature(zone: RCZone) -> str:
    """CasADi name of the indoor air temperature of a zone."""
    return f"{state_instance(zone, 'Ti')}.T"


class _ZoneAssembler:
    def __init__(self, zone: RCZone) -> None:
        self.zone = zone
        self.p = zone.prefix
        self._instances: list[Instance] = []
        self.connections: list[tuple[str, str]] = []

    @property
    def instances(self) -> list[Instance]:
        return self._instances

    def add(self, instance: Instance) -> None:
        self._instances.append(instance.model_copy(update={"zone": self.zone.name}))

    def capacitor(self, node: str, capacitance: float, description: str) -> str:
        name = state_instance(self.zone, node)
        self.add(
            Instance(
                name=name,
                component="Thermal.HeatCapacitor",
                parameters={"C": capacitance, "T0": self.zone.temperature_initial},
                description=description,
            )
        )
        return f"{name}.port"

    def resistor(self, symbol: str, resistance: float, port_a: str, port_b: str) -> None:
        name = f"{self.p}{symbol}"
        self.add(Instance(name=name, component="Thermal.ThermalResistor", parameters={"R": resistance}))
        self.connections += [(port_a, f"{name}.port_a"), (f"{name}.port_b", port_b)]

    def conductor(self, symbol: str, conductance: float, port_a: str, port_b: str) -> None:
        name = f"{self.p}{symbol}"
        self.add(Instance(name=name, component="Thermal.ThermalConductor", parameters={"G": conductance}))
        self.connections += [(port_a, f"{name}.port_a"), (f"{name}.port_b", port_b)]

    def heat_input(self, symbol: str, signal: str, port: str, scale: float = 1.0) -> None:
        name = f"{self.p}{symbol}"
        if scale == 1.0:
            instance = Instance(name=name, component="Sources.PrescribedHeatFlow", inputs={"Q_flow": signal})
        else:
            instance = Instance(
                name=name, component="Sources.ScaledHeatFlow", parameters={"k": scale}, inputs={"u": signal}
            )
        self.add(instance)
        self.connections.append((f"{name}.port", port))

    def solar(self, kind: str, port: str, fraction: float = 1.0, port_kind: str = "") -> None:
        """Solar gains ``fraction*area*irradiance`` of every orientation into ``port``."""
        for aperture in self.zone.solar_apertures:
            area = aperture.window if kind == "gA" else aperture.opaque
            if area <= 0 or fraction <= 0:
                continue
            orientation = aperture.orientation
            self.heat_input(
                f"{_SOLAR_NAMES[kind]}{port_kind}_{orientation.name}",
                orientation.irradiance,
                port,
                scale=fraction * area,
            )


def _r1c1(zone: RCZone, parameters: R1C1Parameters, z: _ZoneAssembler) -> None:
    air = z.capacitor("Ti", parameters.air_capacitance, "Lumped zone heat capacity")
    z.resistor("Ria", parameters.indoor_outdoor_resistance, air, f"{OUTDOOR}.port")
    z.resistor("Rig", parameters.ground_resistance, air, f"{GROUND}.port")
    z.solar("gA", air)
    z.solar("aE", air)
    z.heat_input("intGai", f"{z.p}QInt", air)
    z.heat_input("hea", f"{z.p}QHea", air)


def _envelope(parameters: R3C2Parameters | R4C3Parameters, z: _ZoneAssembler) -> tuple[str, str]:
    air = z.capacitor("Ti", parameters.air_capacitance, "Indoor air and furniture heat capacity")
    envelope = z.capacitor("Te", parameters.envelope_capacitance, "Building envelope and thermal mass heat capacity")
    z.resistor("Rie", parameters.indoor_envelope_resistance, air, envelope)
    z.resistor("Ria", parameters.indoor_outdoor_resistance, air, f"{OUTDOOR}.port")
    z.resistor("Rea", parameters.envelope_outdoor_resistance, envelope, f"{OUTDOOR}.port")
    z.resistor("Reg", parameters.envelope_ground_resistance, envelope, f"{GROUND}.port")
    z.solar("gA", air)
    z.solar("aE", envelope)
    z.heat_input("intGai", f"{z.p}QInt", air)
    return air, envelope


def _r3c2(zone: RCZone, parameters: R3C2Parameters, z: _ZoneAssembler) -> None:
    air, _ = _envelope(parameters, z)
    z.heat_input("hea", f"{z.p}QHea", air)


def _r4c3(zone: RCZone, parameters: R4C3Parameters, z: _ZoneAssembler) -> None:
    air, _ = _envelope(parameters, z)
    emitter = z.capacitor("Th", parameters.emitter_capacitance, "Heat emitter heat capacity")
    z.resistor("Rih", parameters.emitter_resistance, emitter, air)
    z.heat_input("hea", f"{z.p}QHea", emitter)


def _iso13790(zone: RCZone, parameters: ISO13790Parameters, z: _ZoneAssembler) -> None:
    air = z.capacitor("Ti", parameters.air_capacitance, "Indoor air and furniture heat capacity")
    mass = z.capacitor("Tm", parameters.mass_capacitance, "Building thermal mass heat capacity")
    surface = f"{z.p}surface"
    z.add(
        Instance(
            name=surface,
            component="Thermal.StarNode3",
            parameters={
                "G1": parameters.air_surface_conductance,
                "G2": parameters.window_conductance,
                "G3": parameters.surface_mass_conductance,
            },
            description="Massless surface node of ISO 13790, solved in closed form",
        )
    )
    z.connections += [(f"{surface}.port_1", air), (f"{surface}.port_2", f"{OUTDOOR}.port"), (f"{surface}.port_3", mass)]
    z.conductor("Hve", parameters.ventilation_conductance, air, f"{OUTDOOR}.port")
    z.conductor("Hem", parameters.mass_outdoor_conductance, mass, f"{OUTDOOR}.port")
    z.conductor("Hg", parameters.ground_conductance, mass, f"{GROUND}.port")
    radiant = 1 - parameters.convective_fraction
    z.heat_input("intGai", f"{z.p}QInt", air, scale=parameters.convective_fraction)
    if radiant > 0:
        z.heat_input("intGaiSur", f"{z.p}QInt", f"{surface}.node", scale=parameters.surface_fraction * radiant)
        z.heat_input("intGaiMas", f"{z.p}QInt", mass, scale=parameters.mass_fraction * radiant)
    for kind in ("gA", "aE"):
        z.solar(kind, f"{surface}.node", parameters.surface_fraction, port_kind="Sur")
        z.solar(kind, mass, parameters.mass_fraction, port_kind="Mas")
    z.heat_input("hea", f"{z.p}QHea", air)


def connected_rc_model(building: RCBuilding) -> ConnectedModel:
    """``building_mpc`` assembled from components, equivalent to the flat model of ``building``."""
    if building.systems:
        raise NotImplementedError(
            "The FastModel library does not model energy systems yet: use the flat model for "
            f"{[system.name for system in building.systems]}."
        )
    inputs = [TopInput(name="TOut", unit="K", description="Outdoor dry-bulb air temperature")]
    inputs += [
        TopInput(
            name=o.irradiance,
            unit="W/m2",
            description=f"Total solar irradiance, azimuth {o.azimuth:g} deg, tilt {o.tilt:g} deg",
        )
        for o in building.orientations
    ]
    instances = [
        Instance(
            name=OUTDOOR, component="Sources.PrescribedTemperature", inputs={"T": "TOut"}, description="Outdoor air"
        ),
        Instance(
            name=GROUND,
            component="Sources.FixedTemperature",
            parameters={"T": building.ground_temperature},
            description="Ground below the slab",
        ),
    ]
    connections: list[tuple[str, str]] = []
    builders = {"R1C1": _r1c1, "R3C2": _r3c2, "R4C3": _r4c3, "ISO13790": _iso13790}
    for zone in building.zones:
        inputs += [
            TopInput(
                name=f"{zone.prefix}QInt", unit="W", description="Internal heat gains (occupants, appliances, lighting)"
            ),
            TopInput(
                name=f"{zone.prefix}QHea", unit="W", description="Heating (> 0) or cooling (< 0) power, control input"
            ),
        ]
        assembler = _ZoneAssembler(zone)
        builders[zone.parameters.model_type.value](zone, zone.parameters, assembler)  # type: ignore[operator]
        instances += assembler.instances
        connections += assembler.connections
    zones = {zone.name: zone for zone in building.zones}
    for coupling in building.couplings:
        name = f"H_{coupling.zone_a}_{coupling.zone_b}"
        instances.append(
            Instance(
                name=name,
                component="Thermal.ThermalConductor",
                parameters={"G": coupling.conductance},
                description=f"Internal walls between {coupling.zone_a} and {coupling.zone_b}",
            )
        )
        connections += [
            (f"{state_instance(zones[coupling.zone_a], 'Ti')}.port", f"{name}.port_a"),
            (f"{name}.port_b", f"{state_instance(zones[coupling.zone_b], 'Ti')}.port"),
        ]
    return ConnectedModel(
        name=building.name,
        description=f"{len(building.zones)}-zone RC building model ({building.model_type}), connected components",
        inputs=inputs,
        instances=instances,
        connections=connections,
    )
