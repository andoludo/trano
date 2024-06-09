from enum import Enum


class Tilt(Enum):
    wall = "wall"
    ceiling = "ceiling"
    floor = "floor"


class Azimuth:
    north = 0
    south = 90
    east = 45
    west = 135


class Flow(Enum):
    inlet = "inlet"
    outlet = "outlet"
    inlet_or_outlet = "inlet_or_outlet"
    undirected = "undirected"
    interchangeable_port = "interchangeable_port"
