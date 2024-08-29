from trano.models.elements.base import (
    BaseVariant,
)
from trano.models.elements.system import Ventilation

class DamperVariant(BaseVariant):
    complex: str = "complex"



class Damper(Ventilation):
    ...


class VAV(Damper):
    variant: str = DamperVariant.default
