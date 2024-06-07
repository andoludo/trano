from pydantic import Field

from neosim.models.elements.materials.base import BaseData


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


class IdeasConstruction(BaseData):
    template: str = Field(
        default="""      record {{ construction.name }}
    "{{ construction.name }}"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
{#      incLastLay = IDEAS.Types.Tilt.Wall,#}
      mats={
        {%- for layer in construction.layers -%}
        {{ package_name }}.Data.Materials.{{ layer.material.name }}
        (d={{ layer.thickness }}){{ "," if not loop.last }}
        {%- endfor %}
    });
    end {{ construction.name }};"""
    )
