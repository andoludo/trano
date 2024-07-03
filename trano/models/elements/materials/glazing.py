from pydantic import Field

from trano.models.elements.materials.base import BaseData


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
