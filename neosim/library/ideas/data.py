from pydantic import Field

from neosim.library.data.base import BaseConstructionData, BaseData


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


class IdeasMaterial(BaseData):
    template: str = Field(
        default="""
    record {{ construction.name }} = IDEAS.Buildings.Data.Interfaces.Material (
 k={{construction.thermal_conductivity}},
      c={{construction.specific_heat_capacity}},
      rho={{construction.density}},
      epsLw=0.88,
      epsSw=0.55);"""
    )


class IdeasGlazing(BaseData):
    template: str = Field(
        default="""record  {{ construction.name }} = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay={{ construction.layers|length }},
      final checkLowPerformanceGlazing=false,
          mats={
        {%- for layer in construction.layers -%}
        {{ package_name }}.Data.Materials.{{ layer.material.name }}
        (d={{ layer.thickness }}){{ "," if not loop.last }}
        {%- endfor %}
    },
    final SwTrans=[0, 0.721;
                    10, 0.720;
                    20, 0.718;
                    30, 0.711;
                    40, 0.697;
                    50, 0.665;
                    60, 0.596;
                    70, 0.454;
                    80, 0.218;
                    90, 0.000],
      final SwAbs=[0, 0.082, 0, 0.062;
                  10, 0.082, 0, 0.062;
                  20, 0.084, 0, 0.063;
                  30, 0.086, 0, 0.065;
                  40, 0.090, 0, 0.067;
                  50, 0.094, 0, 0.068;
                  60, 0.101, 0, 0.067;
                  70, 0.108, 0, 0.061;
                  80, 0.112, 0, 0.045;
                  90, 0.000, 0, 0.000],
      final SwTransDif=0.619,
      final SwAbsDif={0.093, 0,  0.063},
      final U_value=2.9,
      final g_value=0.78

    ) "{{ package_name }}";"""
    )


class IdeasData(BaseConstructionData):
    template: str = """package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
{% for g in glazing %}
    {{ g|safe }}
{% endfor %}
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;
{%- for m in material -%}
    {{ m|safe }}
{%- endfor %}
end Materials;
package Constructions "Library of building envelope constructions"
{%- for c in construction -%}
    {{ c|safe}}
{%- endfor %}

end Constructions;
end Data;"""
    construction: IdeasConstruction
    material: IdeasMaterial
    glazing: IdeasGlazing
