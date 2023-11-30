{%- macro assign_parameters(values) -%}
    {% raw %}{{% endraw %} {{  values | join(', ') }} {% raw %}}{% endraw %}
{%- endmacro %}
Buildings.ThermalZones.Detailed.MixedAir {{space.name}}(
redeclare package Medium = Medium,
AFlo={{floor_area}},
hRoo={{space.height}},
{%- for boundary in space.boundaries -%}
{%- if boundary.type == 'ExternalWall' and boundary.number -%}
nConExt={{boundary.number}},
datConExt(
layers={{assign_parameters(boundary.layers)}},
A={{assign_parameters(boundary.surfaces)}},
azi={{assign_parameters(boundary.azimuths)}},
til={{assign_parameters(boundary.tilts)}}),
{%- endif %}
{%- if boundary.type == 'InternalElement' and boundary.number -%}
nConPar={{boundary.number}},
datConPar(
layers={{assign_parameters(boundary.layers)}},
A={{assign_parameters(boundary.surfaces)}},
til={{assign_parameters(boundary.tilts)}}),
{%- endif %}
{%- if boundary.type == 'WindowedWall' and boundary.number -%}
nConExtWin={{boundary.number}},
datConExtWin(
layers={{assign_parameters(boundary.layers)}},
glaSys={{assign_parameters(boundary.window_layers)}},
wWin={{ boundary.window_width }},
hWin={{ boundary.window_height }},
A={{assign_parameters(boundary.surfaces)}},
til={{assign_parameters(boundary.tilts)}}),
{%- endif %}
{%- endfor %}
nConBou=0,
nSurBou=0,
energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Floor"
annotation (Placement(transformation(extent=\{\{48,-62},{88,-22\}\})));
