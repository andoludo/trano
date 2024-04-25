from pydantic import computed_field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.models.constants import Azimuth, Tilt
from neosim.models.elements.base import BaseElement


class BaseWall(BaseElement):
    ...

    @computed_field  # type: ignore
    @property
    def length(self) -> int:
        if hasattr(self, "surfaces"):
            return len(self.surfaces)
        return 1


class BaseSimpleWall(BaseWall):
    surface: float | int
    azimuth: float | int
    tilt: Tilt
    construction: Construction | Glass


class ExternalWall(BaseSimpleWall):
    ...


class FloorOnGround(BaseSimpleWall):
    azimuth: float | int = Azimuth.south
    tilt: Tilt = Tilt.floor


class Window(BaseSimpleWall):
    width: float | int
    height: float | int


class WindowedWall(BaseSimpleWall):
    ...


class InternalElement(BaseSimpleWall):
    ...
