from trano.elements.base import BaseElement
from trano.elements.types import ContainerTypes


class BaseBoundary(BaseElement): ...


class Boundary(BaseBoundary):
    container_type: ContainerTypes = "ventilation"
