from trano.models.constants import Azimuth, Tilt
from trano.models.elements.envelope.base import BaseFloorOnGround


class FloorOnGround(BaseFloorOnGround):
    azimuth: float | int = Azimuth.south
    tilt: Tilt = Tilt.floor
