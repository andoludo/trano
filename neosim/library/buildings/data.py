from typing import List, Optional, Union

from pydantic import Field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.library.data.base import BaseConstructionData, BaseData
from neosim.material import Material


class BuildingsConstruction(BaseData):
    template: str = Field(
        default="""    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        {{ construction.name }}(
    final nLay={{ construction.layers|length }},
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={
    {%- for layer in construction.layers -%}
        Buildings.HeatTransfer.Data.Solids.Generic(
        x={{ layer.thickness }},
        k={{ layer.material.thermal_conductivity }},
        c={{ layer.material.specific_heat_capacity }},
        d={{ layer.material.density }}){{ "," if not loop.last }}
    {%- endfor %}
    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={% raw %}{{20,84},{34,98}}{% endraw %})));"""
    )


class BuildingsMaterial(BaseData):
    template: Optional[str] = None
    constructions: List[Union[Construction, Material, Glass]] = Field(default=[])


class BuildingsGlazing(BaseData):
    template: str = Field(
        default="""    {% set glass_layers =    construction.layers |
        selectattr('layer_type', 'eq', "glass") | list %}
    {% set gas_layers =    construction.layers | selectattr('layer_type', 'eq', "gas") | list %}
    parameter Buildings.HeatTransfer.Data.GlazingSystems.Generic {{ construction.name }}(
    final glass={
    {% for layer in glass_layers %}
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x={{ layer.thickness }},
        k={{ layer.material.thermal_conductivity }},
        tauSol={{ macros.join_list(layer.material.solar_transmittance ) }},
        rhoSol_a={{ macros.join_list(layer.material.solar_reflectance_outside_facing ) }},
        rhoSol_b={{ macros.join_list(layer.material.solar_reflectance_room_facing ) }},
        tauIR={{ layer.material.infrared_transmissivity }},
        absIR_a={{ layer.material.infrared_absorptivity_outside_facing }},
        absIR_b={{ layer.material.infrared_absorptivity_room_facing }})
        {{ "," if not loop.last }}
    {% endfor %}
    },
    final gas={
    {% for layer in gas_layers %}
        {% if layer.layer_type == 'gas' %}
            Buildings.HeatTransfer.Data.Gases.Air(x={{ layer.thickness }})
            {{ "," if not loop.last }}
        {% endif %}
    {% endfor %}
    },
    UFra={{ construction.u_value_frame }})
    annotation (
    defaultComponentPrefixes="parameter",
    defaultComponentName="datGlaSys");"""
    )


class BuildingsData(BaseConstructionData):
    template: str = """
{% for g in glazing %}
    {{ g|safe }}
{% endfor %}
{%- for c in construction -%}
    {{ c|safe}}
{%- endfor %}
"""
    construction: BuildingsConstruction
    material: BuildingsMaterial
    glazing: BuildingsGlazing
