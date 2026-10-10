# A walk through the BESTEST cases

The [validation page](bestest.md) lists every KPI of every case. This page takes a few cases and
tells, for each of them, what the case is, what the six reference programs of ASHRAE Standard
140-2020 (BSIMAC, CSE, DeST, EnergyPlus, ESP-r and TRNSYS) expect, how the trano YAML describes
the case and how the results of each library compare with that expectation. The YAML fragments
are cut out of the generated case files in `validation/bestest/cases/`, the tables and the charts
are rendered from the reference data and the last results, with
`python -m validation.bestest report --docs`.

A band is the range a result must fall in: the acceptance limits of the standard for the annual
loads, and the spread of the reference programs widened by 5 % of the largest peak (loads) or by
1 K (temperatures) otherwise. In the result tables, ✓ marks a value inside its band, ~ a known
deviation documented on the validation page and ✗ a value outside its band. Buildings and IDEAS
gate the test suite; the reduced-order (AixLib) and ISO 13790 zones are shown for information.

## Case 600: Base case, low mass

The base case: a single rectangular zone of 8 m by 6 m by 2.7 m in Denver (cold, sunny, 1609 m above sea level) with light-weight walls (wood siding, fibreglass, plasterboard), a light roof and a timber floor over 1 m of insulation whose underside sees the outdoor air. The south wall carries 12 m2 of clear double glazing without any frame. Infiltration is 0.414 air changes per hour (0.5 at sea level, corrected for the altitude), the internal gains are 200 W around the clock (60 % radiant, 40 % convective) and an ideal system keeps the air between 20 degC (heating) and 27 degC (cooling) with unlimited capacity.

### What the reference programs expect

| KPI | BSIMAC | CSE | DeST | EnergyPlus | ESP-r | TRNSYS | Band |
|---|---|---|---|---|---|---|---|
| annual heating [MWh] | 4.050 | 3.993 | 4.047 | 4.324 | 4.362 | 4.504 | 3.750 to 4.980 (limits of the standard) |
| annual cooling [MWh] | 5.822 | 5.913 | 5.432 | 6.027 | 6.162 | 5.780 | 5.000 to 6.830 (limits of the standard) |
| peak heating [kW] | 3.255 | 3.020 | 3.035 | 3.204 | 3.228 | 3.359 | 2.852 to 3.527 (programs ± tolerance) |
| peak cooling [kW] | 5.650 | 6.481 | 5.422 | 6.352 | 6.193 | 6.046 | 5.098 to 6.805 (programs ± tolerance) |

Winter nights drive the heating, so the heating peak falls on the coldest hours of January or December. The cooling peak falls on a sunny winter day too: the low sun pours through the south glazing and the light envelope has nothing to store it in, so the zone hits 27 degC within a few hours. The annual loads must fall inside the acceptance limits of the standard; the peaks must fall inside the spread of the reference programs widened by 5 % of the largest peak. The hourly loads of 1 February are compared as well: a cold clear day on which the zone switches from heating at night to cooling around noon.

### How the YAML describes it

The zone is a `space` with the `infiltration` variant, whose `ach` parameter is the air change rate; `linearize_emissive_power: false` keeps the long-wave exchange non-linear as the standard asks. The floor is a `floor_on_grounds` boundary with the `outdoor_air` variant (its outer surface follows the outdoor dry-bulb temperature), the roof is an external wall with the `ceiling` tilt. The 200 W of internal gains are an `occupancy` element occupied all day, with the radiant, convective and latent gains per square metre of floor. The ideal system is the `ideal_heating_cooling` emission: its set points are day schedules in kelvin (time since midnight in seconds, value) repeated every day, and its capacities are 1 MW so that the set points are always met. The weather is the Denver TMY3 file shipped with Buildings; the atmospheric pressure is read from the file, so that the air density, and with it the infiltration mass flow, is that of the site. Every material has 18 states per 0.2 m of layer (`number_of_states`), the discretisation of Buildings' own BESTEST models.

`weather`:

```yaml
parameters:
  path: Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos")
  atmospheric_pressure_source: Buildings.BoundaryConditions.Types.DataSource.File
```

`constructions[id=LIGHT_WALL:001]`:

```yaml
id: LIGHT_WALL:001
layers:
- material: WOOD_SIDING:001
  thickness: 0.009
- material: FIBERGLASS:001
  thickness: 0.066
- material: PLASTERBOARD:001
  thickness: 0.012
```

`spaces[0].parameters`:

```yaml
floor_area: 48.0
average_room_height: 2.7
ach: 0.414
linearize_emissive_power: 'false'
```

`spaces[0].external_boundaries.windows`:

```yaml
- surface: 12.0
  azimuth: 0.0
  tilt: wall
  construction: DOUBLE_CLEAR:001
  width: 6.0
  height: 2.0
  frame_fraction: 0.001
```

`spaces[0].occupancy`:

```yaml
parameters:
  occupancy: '{1, 86400}'
  gain: '[120/48; 80/48; 0]'
  heat_gain_if_occupied: '1'
```

`spaces[0].emissions`:

```yaml
- ideal_heating_cooling:
    id: HVAC:001
    parameters:
      heating_setpoint_schedule: '[0, 293.15]'
      cooling_setpoint_schedule: '[0, 300.15]'
      maximum_heating_power: 1000000.0
      maximum_cooling_power: 1000000.0
```

### How trano fares

| KPI | Band | Buildings | IDEAS | ISO 13790 | reduced order |
|---|---|---|---|---|---|
| annual heating [MWh] | 3.750 to 4.980 | 4.449 ✓ | 4.541 ✓ | 3.122 ✗ | 4.469 ✓ |
| annual cooling [MWh] | 5.000 to 6.830 | 5.973 ✓ | 6.265 ✓ | 5.298 ✓ | 4.267 ✗ |
| peak heating [kW] | 2.852 to 3.527 | 3.215 ✓ | 3.314 ✓ | 3.099 ✓ | 3.020 ✓ |
| peak cooling [kW] | 5.098 to 6.805 | 6.187 ✓ | 6.545 ✓ | 4.870 ✗ | 4.180 ✗ |

- Buildings: 4 of 4 KPIs inside their band.
- IDEAS: 4 of 4 KPIs inside their band.
- ISO 13790: 2 of 4 KPIs inside their band, outside: annual heating 3.122 MWh against 3.750 to 4.980; peak cooling 4.870 kW against 5.098 to 6.805.
- reduced order: 2 of 4 KPIs inside their band, outside: annual cooling 4.267 MWh against 5.000 to 6.830; peak cooling 4.180 kW against 5.098 to 6.805.

<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 320" width="100%" role="img" aria-label="Case 600: hourly load on 1 February" style="font-family: sans-serif; font-size: 12px; max-width: 720px">
<title>Case 600: hourly load on 1 February</title>
<text x="56" y="14" font-weight="bold">Case 600: hourly load on 1 February</text>
<line x1="56" x2="704" y1="280.0" y2="280.0" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="284.0" text-anchor="end">-6</text>
<line x1="56" x2="704" y1="219.0" y2="219.0" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="223.0" text-anchor="end">-4</text>
<line x1="56" x2="704" y1="157.9" y2="157.9" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="161.9" text-anchor="end">-2</text>
<line x1="56" x2="704" y1="96.9" y2="96.9" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="100.9" text-anchor="end">0</text>
<line x1="56" x2="704" y1="35.9" y2="35.9" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="39.9" text-anchor="end">2</text>
<text x="56.0" y="296" text-anchor="middle">1</text>
<text x="140.5" y="296" text-anchor="middle">4</text>
<text x="225.0" y="296" text-anchor="middle">7</text>
<text x="309.6" y="296" text-anchor="middle">10</text>
<text x="394.1" y="296" text-anchor="middle">13</text>
<text x="478.6" y="296" text-anchor="middle">16</text>
<text x="563.1" y="296" text-anchor="middle">19</text>
<text x="647.7" y="296" text-anchor="middle">22</text>
<text x="380.0" y="314" text-anchor="middle">hour of the day</text>
<text transform="translate(14 150.0) rotate(-90)" text-anchor="middle">load [kWh], heating positive</text>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,35.0 84.2,34.6 112.3,32.2 140.5,28.8 168.7,26.4 196.9,24.9 225.0,24.0 253.2,32.8 281.4,64.9 309.6,92.6 337.7,99.0 365.9,146.3 394.1,205.5 422.3,219.9 450.4,214.1 478.6,195.8 506.8,164.0 535.0,120.7 563.1,96.9 591.3,96.9 619.5,84.4 647.7,58.1 675.8,41.7 704.0,36.2"><title>BSIMAC</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,39.5 84.2,36.2 112.3,34.3 140.5,32.2 168.7,31.3 196.9,31.0 225.0,30.4 253.2,42.6 281.4,80.1 309.6,96.9 337.7,125.0 365.9,190.6 394.1,230.0 422.3,239.4 450.4,220.5 478.6,185.1 506.8,137.5 535.0,98.1 563.1,95.4 591.3,70.4 619.5,49.6 647.7,37.7 675.8,30.7 704.0,27.6"><title>CSE</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,36.2 84.2,33.7 112.3,31.9 140.5,29.5 168.7,28.5 196.9,27.6 225.0,27.3 253.2,32.2 281.4,60.3 309.6,96.9 337.7,107.9 365.9,164.3 394.1,202.8 422.3,219.6 450.4,219.6 478.6,203.1 506.8,167.7 535.0,116.7 563.1,96.9 591.3,92.3 619.5,63.3 647.7,45.9 675.8,35.6 704.0,29.8"><title>DeST</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,31.3 84.2,29.2 112.3,27.3 140.5,25.8 168.7,25.2 196.9,25.2 225.0,24.9 253.2,39.5 281.4,81.9 309.6,96.9 337.7,136.9 365.9,201.0 394.1,239.4 422.3,247.3 450.4,224.2 478.6,186.3 506.8,134.7 535.0,97.5 563.1,94.2 591.3,67.0 619.5,46.9 647.7,34.0 675.8,26.1 704.0,22.4"><title>EnergyPlus</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,33.4 84.2,29.2 112.3,26.7 140.5,24.3 168.7,23.1 196.9,22.7 225.0,21.8 253.2,33.1 281.4,72.8 309.6,96.9 337.7,136.0 365.9,199.4 394.1,240.0 422.3,251.9 450.4,230.9 478.6,195.8 506.8,145.4 535.0,100.6 563.1,93.8 591.3,66.4 619.5,45.0 647.7,32.8 675.8,24.9 704.0,21.5"><title>ESP-r</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,30.7 84.2,27.6 112.3,25.5 140.5,23.4 168.7,22.4 196.9,22.1 225.0,21.8 253.2,43.5 281.4,87.7 309.6,101.8 337.7,138.7 365.9,195.5 394.1,231.8 422.3,237.3 450.4,217.7 478.6,183.3 506.8,132.3 535.0,96.9 563.1,85.9 591.3,65.5 619.5,45.9 647.7,32.5 675.8,24.0 704.0,20.0"><title>TRNSYS</title></polyline>
<polyline fill="none" stroke="#1f77b4" stroke-width="2.5" points="56.0,32.1 84.2,28.9 112.3,27.3 140.5,25.6 168.7,24.5 196.9,24.5 225.0,24.5 253.2,37.4 281.4,77.1 309.6,96.9 337.7,135.9 365.9,199.5 394.1,235.3 422.3,246.8 450.4,227.0 478.6,191.1 506.8,141.8 535.0,99.1 563.1,95.5 591.3,68.7 619.5,46.8 647.7,34.6 675.8,26.2 704.0,22.5"><title>Buildings</title></polyline>
<polyline fill="none" stroke="#d62728" stroke-width="2.5" points="56.0,29.7 84.2,27.4 112.3,25.6 140.5,23.8 168.7,22.6 196.9,22.7 225.0,22.6 253.2,34.8 281.4,75.4 309.6,96.9 337.7,136.4 365.9,201.1 394.1,240.2 422.3,252.5 450.4,231.5 478.6,195.4 506.8,145.2 535.0,99.5 563.1,95.4 591.3,66.6 619.5,44.1 647.7,32.4 675.8,24.3 704.0,20.7"><title>IDEAS</title></polyline>
<line x1="358.0" x2="382.0" y1="10" y2="10" stroke="#9aa5b1" stroke-width="1.0"/>
<text x="388.0" y="14">reference programs</text>
<line x1="521.0" x2="545.0" y1="10" y2="10" stroke="#1f77b4" stroke-width="2.5"/>
<text x="551.0" y="14">Buildings</text>
<line x1="625.5" x2="649.5" y1="10" y2="10" stroke="#d62728" stroke-width="2.5"/>
<text x="655.5" y="14">IDEAS</text>
</svg>

## Case 600FF: 600 free floating

Case 600 without any heating or cooling: the temperature of the zone floats freely under the weather and the internal gains.

### What the reference programs expect

| KPI | BSIMAC | CSE | DeST | EnergyPlus | ESP-r | TRNSYS | Band |
|---|---|---|---|---|---|---|---|
| maximum temperature [degC] | 63.400 | 68.400 | 65.000 | 63.800 | 64.600 | 62.400 | 61.400 to 69.400 (programs ± tolerance) |
| minimum temperature [degC] | -9.900 | -12.900 | -13.500 | -12.600 | -13.500 | -13.800 | -14.800 to -8.900 (programs ± tolerance) |
| mean temperature [degC] | 26.100 | 25.600 | 25.300 | 24.900 | 25.300 | 24.300 | 23.300 to 27.100 (programs ± tolerance) |

The light envelope stores next to nothing, so the zone follows the weather closely: it drops well below freezing on winter nights and climbs above 60 degC on sunny days, with the glazing turning the zone into a greenhouse. The standard compares the maximum, minimum and annual mean temperature with the spread of the reference programs (widened by 1 K here), and the hourly temperatures of 1 February.

### How the YAML describes it

The YAML is that of case 600 without the `emissions` list: a space without an emission is free-floating. The occupancy stays, since the 200 W of internal gains are part of the case.

`spaces[0].parameters`:

```yaml
floor_area: 48.0
average_room_height: 2.7
ach: 0.414
linearize_emissive_power: 'false'
```

`spaces[0].occupancy`:

```yaml
parameters:
  occupancy: '{1, 86400}'
  gain: '[120/48; 80/48; 0]'
  heat_gain_if_occupied: '1'
```

### How trano fares

| KPI | Band | Buildings | IDEAS | ISO 13790 | reduced order |
|---|---|---|---|---|---|
| maximum temperature [degC] | 61.400 to 69.400 | 63.318 ✓ | 66.086 ✓ | 54.174 ✗ | 68.891 ✓ |
| minimum temperature [degC] | -14.800 to -8.900 | -12.847 ✓ | -13.193 ✓ | -7.447 ✗ | -13.741 ✓ |
| mean temperature [degC] | 23.300 to 27.100 | 24.664 ✓ | 25.109 ✓ | 26.564 ✓ | 22.650 ✗ |

- Buildings: 3 of 3 KPIs inside their band.
- IDEAS: 3 of 3 KPIs inside their band.
- ISO 13790: 1 of 3 KPIs inside their band, outside: maximum temperature 54.174 degC against 61.400 to 69.400; minimum temperature -7.447 degC against -14.800 to -8.900.
- reduced order: 2 of 3 KPIs inside their band, outside: mean temperature 22.650 degC against 23.300 to 27.100.

<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 320" width="100%" role="img" aria-label="Case 600FF: zone temperature on 1 February" style="font-family: sans-serif; font-size: 12px; max-width: 720px">
<title>Case 600FF: zone temperature on 1 February</title>
<text x="56" y="14" font-weight="bold">Case 600FF: zone temperature on 1 February</text>
<line x1="56" x2="704" y1="263.8" y2="263.8" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="267.8" text-anchor="end">0</text>
<line x1="56" x2="704" y1="165.9" y2="165.9" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="169.9" text-anchor="end">20</text>
<line x1="56" x2="704" y1="68.0" y2="68.0" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="72.0" text-anchor="end">40</text>
<text x="56.0" y="296" text-anchor="middle">1</text>
<text x="140.5" y="296" text-anchor="middle">4</text>
<text x="225.0" y="296" text-anchor="middle">7</text>
<text x="309.6" y="296" text-anchor="middle">10</text>
<text x="394.1" y="296" text-anchor="middle">13</text>
<text x="478.6" y="296" text-anchor="middle">16</text>
<text x="563.1" y="296" text-anchor="middle">19</text>
<text x="647.7" y="296" text-anchor="middle">22</text>
<text x="380.0" y="314" text-anchor="middle">hour of the day</text>
<text transform="translate(14 150.0) rotate(-90)" text-anchor="middle">temperature [degC]</text>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,200.2 84.2,213.4 112.3,225.2 140.5,235.9 168.7,245.2 196.9,253.6 225.0,260.4 253.2,263.4 281.4,255.5 309.6,234.0 337.7,202.6 365.9,164.9 394.1,124.3 422.3,87.1 450.4,58.7 478.6,41.5 506.8,37.1 535.0,48.4 563.1,70.4 591.3,95.4 619.5,120.4 647.7,143.4 675.8,164.9 704.0,184.0"><title>BSIMAC</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,228.1 84.2,236.9 112.3,245.2 140.5,252.1 168.7,258.5 196.9,263.8 225.0,268.7 253.2,268.7 281.4,251.6 309.6,217.3 337.7,175.7 365.9,132.1 394.1,89.5 422.3,54.3 450.4,33.2 478.6,27.8 506.8,39.1 535.0,68.0 563.1,100.8 591.3,129.2 619.5,153.7 647.7,175.2 675.8,194.8 704.0,211.0"><title>CSE</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,241.8 84.2,249.6 112.3,257.0 140.5,263.8 168.7,269.7 196.9,275.1 225.0,279.5 253.2,280.0 281.4,264.8 309.6,230.5 337.7,187.5 365.9,140.9 394.1,96.9 422.3,62.1 450.4,38.1 478.6,28.3 506.8,35.7 535.0,62.1 563.1,94.4 591.3,123.3 619.5,148.3 647.7,170.8 675.8,190.9 704.0,207.5"><title>DeST</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,237.4 84.2,245.2 112.3,252.1 140.5,258.5 168.7,263.8 196.9,268.2 225.0,272.2 253.2,271.2 281.4,250.6 309.6,215.4 337.7,172.3 365.9,126.7 394.1,84.6 422.3,51.3 450.4,33.7 478.6,30.8 506.8,44.0 535.0,74.4 563.1,106.2 591.3,135.1 619.5,158.6 647.7,180.6 675.8,198.7 704.0,213.9"><title>EnergyPlus</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,234.5 84.2,243.8 112.3,251.6 140.5,258.9 168.7,265.3 196.9,270.7 225.0,275.1 253.2,275.6 281.4,256.0 309.6,218.8 337.7,173.3 365.9,126.3 394.1,80.7 422.3,44.0 450.4,24.9 478.6,20.0 506.8,32.2 535.0,63.6 563.1,99.8 591.3,129.7 619.5,155.6 647.7,178.6 675.8,198.7 704.0,215.9"><title>ESP-r</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,239.8 84.2,248.2 112.3,255.5 140.5,261.9 168.7,267.3 196.9,272.2 225.0,276.1 253.2,270.2 281.4,243.8 309.6,205.1 337.7,162.5 365.9,117.9 394.1,76.8 422.3,47.4 450.4,33.7 478.6,34.2 506.8,50.8 535.0,85.1 563.1,118.4 591.3,145.3 619.5,169.3 647.7,190.4 675.8,209.0 704.0,224.2"><title>TRNSYS</title></polyline>
<polyline fill="none" stroke="#1f77b4" stroke-width="2.5" points="56.0,238.2 84.2,246.2 112.3,253.5 140.5,260.0 168.7,265.5 196.9,270.3 225.0,274.2 253.2,271.8 281.4,253.0 309.6,217.4 337.7,173.5 365.9,128.3 394.1,85.9 422.3,51.8 450.4,31.9 478.6,27.4 506.8,39.4 535.0,67.3 563.1,101.1 591.3,131.1 619.5,156.5 647.7,178.7 675.8,198.2 704.0,214.7"><title>Buildings</title></polyline>
<polyline fill="none" stroke="#d62728" stroke-width="2.5" points="56.0,238.4 84.2,246.7 112.3,254.0 140.5,260.5 168.7,266.0 196.9,270.7 225.0,274.5 253.2,272.2 281.4,254.3 309.6,217.7 337.7,173.8 365.9,127.8 394.1,83.8 422.3,47.7 450.4,26.1 478.6,21.0 506.8,32.2 535.0,62.0 563.1,97.0 591.3,127.9 619.5,154.3 647.7,177.3 675.8,197.5 704.0,214.6"><title>IDEAS</title></polyline>
<line x1="358.0" x2="382.0" y1="10" y2="10" stroke="#9aa5b1" stroke-width="1.0"/>
<text x="388.0" y="14">reference programs</text>
<line x1="521.0" x2="545.0" y1="10" y2="10" stroke="#1f77b4" stroke-width="2.5"/>
<text x="551.0" y="14">Buildings</text>
<line x1="625.5" x2="649.5" y1="10" y2="10" stroke="#d62728" stroke-width="2.5"/>
<text x="655.5" y="14">IDEAS</text>
</svg>

## Case 900: Base case, high mass

Case 600 with heavy constructions: the walls are concrete blocks behind foam insulation and wood siding, the floor is a concrete slab over the insulation. The U-values are the same as in case 600; only the thermal mass changes.

### What the reference programs expect

| KPI | BSIMAC | CSE | DeST | EnergyPlus | ESP-r | TRNSYS | Band |
|---|---|---|---|---|---|---|---|
| annual heating [MWh] | 1.726 | 1.379 | 1.591 | 1.664 | 1.585 | 1.814 | 1.040 to 2.280 (limits of the standard) |
| annual cooling [MWh] | 2.714 | 2.464 | 2.383 | 2.489 | 2.488 | 2.267 | 2.350 to 2.600 (limits of the standard) |
| peak heating [kW] | 2.551 | 2.443 | 2.453 | 2.687 | 2.633 | 2.778 | 2.304 to 2.917 (programs ± tolerance) |
| peak cooling [kW] | 3.039 | 3.376 | 2.556 | 3.040 | 2.896 | 2.940 | 2.387 to 3.545 (programs ± tolerance) |

The mass stores the solar gains of the day and gives them back at night, so both the heating and the cooling loads drop a lot against case 600, the cooling by more than half. The peaks drop too, and the heating peak moves to the end of long cold spells rather than to the first cold night. The hourly loads of 1 February show a much smoother profile than case 600.

### How the YAML describes it

Only the constructions change: the walls and the floor refer to the heavy constructions, whose concrete layers carry the mass. For the libraries that aggregate the envelope (the reduced-order and ISO 13790 zones), trano derives the mass class of the zone from the heat capacity of these layers.

`constructions[id=HEAVY_WALL:001]`:

```yaml
id: HEAVY_WALL:001
layers:
- material: WOOD_SIDING:001
  thickness: 0.009
- material: FOAM_INSULATION:001
  thickness: 0.0615
- material: CONCRETE_BLOCK:001
  thickness: 0.1
```

`constructions[id=HEAVY_FLOOR:001]`:

```yaml
id: HEAVY_FLOOR:001
layers:
- material: FLOOR_INSULATION:001
  thickness: 1.007
- material: CONCRETE_SLAB:001
  thickness: 0.08
```

### How trano fares

| KPI | Band | Buildings | IDEAS | ISO 13790 | reduced order |
|---|---|---|---|---|---|
| annual heating [MWh] | 1.040 to 2.280 | 1.718 ✓ | 1.763 ✓ | 1.803 ✓ | 3.876 ✗ |
| annual cooling [MWh] | 2.350 to 2.600 | 2.395 ✓ | 2.599 ✓ | 3.581 ✗ | 3.558 ✗ |
| peak heating [kW] | 2.304 to 2.917 | 2.668 ✓ | 2.765 ✓ | 2.880 ✓ | 3.016 ✗ |
| peak cooling [kW] | 2.387 to 3.545 | 2.974 ✓ | 3.171 ✓ | 3.532 ✓ | 3.655 ✗ |

- Buildings: 4 of 4 KPIs inside their band.
- IDEAS: 4 of 4 KPIs inside their band.
- ISO 13790: 3 of 4 KPIs inside their band, outside: annual cooling 3.581 MWh against 2.350 to 2.600.
- reduced order: 0 of 4 KPIs inside their band, outside: annual heating 3.876 MWh against 1.040 to 2.280; annual cooling 3.558 MWh against 2.350 to 2.600; peak heating 3.016 kW against 2.304 to 2.917; peak cooling 3.655 kW against 2.387 to 3.545.

<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 320" width="100%" role="img" aria-label="Case 900: hourly load on 1 February" style="font-family: sans-serif; font-size: 12px; max-width: 720px">
<title>Case 900: hourly load on 1 February</title>
<text x="56" y="14" font-weight="bold">Case 900: hourly load on 1 February</text>
<line x1="56" x2="704" y1="246.3" y2="246.3" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="250.3" text-anchor="end">0</text>
<line x1="56" x2="704" y1="175.6" y2="175.6" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="179.6" text-anchor="end">0.5</text>
<line x1="56" x2="704" y1="104.9" y2="104.9" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="108.9" text-anchor="end">1</text>
<line x1="56" x2="704" y1="34.1" y2="34.1" stroke="#e0e0e0" stroke-width="1"/>
<text x="50" y="38.1" text-anchor="end">1.5</text>
<text x="56.0" y="296" text-anchor="middle">1</text>
<text x="140.5" y="296" text-anchor="middle">4</text>
<text x="225.0" y="296" text-anchor="middle">7</text>
<text x="309.6" y="296" text-anchor="middle">10</text>
<text x="394.1" y="296" text-anchor="middle">13</text>
<text x="478.6" y="296" text-anchor="middle">16</text>
<text x="563.1" y="296" text-anchor="middle">19</text>
<text x="647.7" y="296" text-anchor="middle">22</text>
<text x="380.0" y="314" text-anchor="middle">hour of the day</text>
<text transform="translate(14 150.0) rotate(-90)" text-anchor="middle">load [kWh], heating positive</text>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,134.6 84.2,114.8 112.3,96.4 140.5,78.0 168.7,62.4 196.9,46.9 225.0,28.5 253.2,41.2 281.4,144.5 309.6,225.1 337.7,246.3 365.9,246.3 394.1,246.3 422.3,246.3 450.4,264.7 478.6,264.7 506.8,246.3 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,246.3 675.8,220.9 704.0,198.2"><title>BSIMAC</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,144.5 84.2,127.5 112.3,111.9 140.5,97.8 168.7,87.9 196.9,78.0 225.0,68.1 253.2,92.1 281.4,169.9 309.6,229.4 337.7,246.3 365.9,246.3 394.1,246.3 422.3,246.3 450.4,249.2 478.6,247.7 506.8,246.3 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,246.3 675.8,235.0 704.0,205.3"><title>CSE</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,95.0 84.2,80.8 112.3,66.7 140.5,52.5 168.7,42.6 196.9,31.3 225.0,24.2 253.2,27.1 281.4,76.6 309.6,162.9 337.7,246.3 365.9,246.3 394.1,246.3 422.3,246.3 450.4,246.3 478.6,246.3 506.8,246.3 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,246.3 675.8,240.7 704.0,212.4"><title>DeST</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,107.7 84.2,93.6 112.3,79.4 140.5,63.9 168.7,52.5 196.9,42.6 225.0,34.1 253.2,59.6 281.4,134.6 309.6,208.1 337.7,246.3 365.9,246.3 394.1,246.3 422.3,246.3 450.4,253.4 478.6,250.6 506.8,246.3 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,246.3 675.8,220.9 704.0,189.7"><title>EnergyPlus</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,117.6 84.2,96.4 112.3,80.8 140.5,66.7 168.7,53.9 196.9,44.0 225.0,32.7 253.2,45.5 281.4,114.8 309.6,192.6 337.7,244.9 365.9,246.3 394.1,246.3 422.3,246.3 450.4,254.8 478.6,257.6 506.8,246.3 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,246.3 675.8,239.3 704.0,206.7"><title>ESP-r</title></polyline>
<polyline fill="none" stroke="#9aa5b1" stroke-width="1.0" points="56.0,100.6 84.2,83.7 112.3,68.1 140.5,51.1 168.7,39.8 196.9,29.9 225.0,20.0 253.2,53.9 281.4,136.0 309.6,211.0 337.7,246.3 365.9,246.3 394.1,246.3 422.3,246.3 450.4,247.7 478.6,246.3 506.8,246.3 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,232.2 675.8,202.5 704.0,172.8"><title>TRNSYS</title></polyline>
<polyline fill="none" stroke="#1f77b4" stroke-width="2.5" points="56.0,105.9 84.2,86.0 112.3,72.4 140.5,58.6 168.7,46.7 196.9,37.1 225.0,28.7 253.2,47.9 281.4,118.7 309.6,205.2 337.7,246.3 365.9,246.3 394.1,246.3 422.3,246.3 450.4,258.0 478.6,257.2 506.8,246.3 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,246.3 675.8,230.5 704.0,195.6"><title>Buildings</title></polyline>
<polyline fill="none" stroke="#d62728" stroke-width="2.5" points="56.0,96.6 84.2,79.3 112.3,65.1 140.5,50.6 168.7,38.5 196.9,29.6 225.0,20.8 253.2,41.3 281.4,116.2 309.6,200.8 337.7,246.2 365.9,246.3 394.1,246.3 422.3,246.4 450.4,270.1 478.6,280.0 506.8,252.4 535.0,246.3 563.1,246.3 591.3,246.3 619.5,246.3 647.7,246.3 675.8,240.1 704.0,201.7"><title>IDEAS</title></polyline>
<line x1="358.0" x2="382.0" y1="10" y2="10" stroke="#9aa5b1" stroke-width="1.0"/>
<text x="388.0" y="14">reference programs</text>
<line x1="521.0" x2="545.0" y1="10" y2="10" stroke="#1f77b4" stroke-width="2.5"/>
<text x="551.0" y="14">Buildings</text>
<line x1="625.5" x2="649.5" y1="10" y2="10" stroke="#d62728" stroke-width="2.5"/>
<text x="655.5" y="14">IDEAS</text>
</svg>

## Case 640: 600 with a night heating setback

Case 600 with a night set-back of the heating: the heating set point is 10 degC from 23:00 to 07:00 and 20 degC otherwise. The cooling set point stays at 27 degC.

### What the reference programs expect

| KPI | BSIMAC | CSE | DeST | EnergyPlus | ESP-r | TRNSYS | Band |
|---|---|---|---|---|---|---|---|
| annual heating [MWh] | 2.682 | 2.403 | 2.619 | 2.662 | 2.654 | 2.653 | 1.580 to 3.760 (limits of the standard) |
| annual cooling [MWh] | 5.804 | 5.644 | 5.237 | 5.763 | 5.893 | 5.477 | 4.440 to 6.860 (limits of the standard) |
| peak heating [kW] | 4.633 | 4.222 | 4.658 | 4.559 | 4.101 | 4.039 | 3.806 to 4.891 (programs ± tolerance) |
| peak cooling [kW] | 5.650 | 6.429 | 5.365 | 6.297 | 6.127 | 5.967 | 5.044 to 6.750 (programs ± tolerance) |

The zone cools down at night, so the annual heating drops against case 600. The heating peak rises though: at 07:00 the system has to bring a cold zone back to 20 degC at once, and this morning pick-up is the largest load of the year, well above the peak of case 600.

### How the YAML describes it

The heating set point becomes a day schedule with the set-back: pairs of time since midnight and set point in kelvin, held constant between two rows, with the step written as two rows at the same time. The schedule repeats every day.

`spaces[0].emissions`:

```yaml
- ideal_heating_cooling:
    id: HVAC:001
    parameters:
      heating_setpoint_schedule: '[0, 283.15; 25200, 283.15; 28800, 293.15; 82800, 293.15; 82800, 283.15;
        86400, 283.15]'
      cooling_setpoint_schedule: '[0, 300.15]'
      maximum_heating_power: 1000000.0
      maximum_cooling_power: 1000000.0
```

### How trano fares

| KPI | Band | Buildings | IDEAS | ISO 13790 | reduced order |
|---|---|---|---|---|---|
| annual heating [MWh] | 1.580 to 3.760 | 2.715 ✓ | 2.769 ✓ | 1.879 ✓ | 2.789 ✓ |
| annual cooling [MWh] | 4.440 to 6.860 | 5.725 ✓ | 6.009 ✓ | 4.942 ✓ | 4.166 ✗ |
| peak heating [kW] | 3.806 to 4.891 | 4.356 ✓ | 4.370 ✓ | 4.350 ✓ | 2.753 ✗ |
| peak cooling [kW] | 5.044 to 6.750 | 6.138 ✓ | 6.491 ✓ | 4.870 ✗ | 4.126 ✗ |

- Buildings: 4 of 4 KPIs inside their band.
- IDEAS: 4 of 4 KPIs inside their band.
- ISO 13790: 3 of 4 KPIs inside their band, outside: peak cooling 4.870 kW against 5.044 to 6.750.
- reduced order: 1 of 4 KPIs inside their band, outside: annual cooling 4.166 MWh against 4.440 to 6.860; peak heating 2.753 kW against 3.806 to 4.891; peak cooling 4.126 kW against 5.044 to 6.750.

## Case 650: 600 with night ventilation and no heating

Case 600 without heating: the cooling set point is 27 degC from 07:00 to 18:00 and off otherwise, and a fan blows 1700 m3/h of outdoor air through the zone from 18:00 to 07:00 without adding any heat.

### What the reference programs expect

| KPI | BSIMAC | CSE | DeST | EnergyPlus | ESP-r | TRNSYS | Band |
|---|---|---|---|---|---|---|---|
| annual heating [MWh] | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 to 0.000 (limits of the standard) |
| annual cooling [MWh] | 4.629 | 4.654 | 4.186 | 4.817 | 4.945 | 4.632 | 3.460 to 5.880 (limits of the standard) |
| peak heating [kW] | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 to 0.000 (programs ± tolerance) |
| peak cooling [kW] | 5.648 | 6.290 | 5.045 | 6.138 | 5.961 | 5.797 | 4.731 to 6.604 (programs ± tolerance) |

The annual heating is zero by construction. The night ventilation flushes the zone with cold outdoor air, so the cooling drops against case 600 but by less than the free cooling would suggest: the zone is light and heats up again as soon as the sun rises. The cooling peak stays close to that of case 600, since it occurs in the afternoon, long after the fan stopped.

### How the YAML describes it

The fan is the `ventilation_schedule` parameter of the space: a day schedule of outdoor air mass flow in kg/s brought in on top of the infiltration, at the outdoor temperature and humidity, 1409 kg/h being 1700 m3/h at the air density of the site. The heating is switched off with a set point of 0 degC and a capacity of zero; the cooling set point is 100 degC outside the cooling hours.

`spaces[0].parameters`:

```yaml
floor_area: 48.0
average_room_height: 2.7
ach: 0.414
linearize_emissive_power: 'false'
ventilation_schedule: '[0, 0.391389; 25200, 0.391389; 25200, 0; 64800, 0; 64800, 0.391389; 86400, 0.391389]'
```

`spaces[0].emissions`:

```yaml
- ideal_heating_cooling:
    id: HVAC:001
    parameters:
      heating_setpoint_schedule: '[0, 273.15]'
      cooling_setpoint_schedule: '[0, 373.15; 25200, 373.15; 25200, 300.15; 64800, 300.15; 64800, 373.15;
        86400, 373.15]'
      maximum_heating_power: 0.0
      maximum_cooling_power: 1000000.0
```

### How trano fares

| KPI | Band | Buildings | IDEAS |
|---|---|---|---|
| annual heating [MWh] | 0.000 to 0.000 | 0.000 ✓ | 0.000 ✓ |
| annual cooling [MWh] | 3.460 to 5.880 | 4.802 ✓ | 5.067 ✓ |
| peak heating [kW] | 0.000 to 0.000 | 0.000 ✓ | 0.000 ✓ |
| peak cooling [kW] | 4.731 to 6.604 | 5.949 ✓ | 6.324 ✓ |

- Buildings: 4 of 4 KPIs inside their band.
- IDEAS: 4 of 4 KPIs inside their band.
- ISO 13790: the case is not supported.
- reduced order: the case is not supported.

## Case 610: 600 with a south overhang

Case 600 with a 1 m deep horizontal overhang above the south window, placed 0.5 m above the top of the glazing and running the full width of the wall.

### What the reference programs expect

| KPI | BSIMAC | CSE | DeST | EnergyPlus | ESP-r | TRNSYS | Band |
|---|---|---|---|---|---|---|---|
| annual heating [MWh] | 4.163 | 4.066 | 4.144 | 4.375 | 4.527 | 4.592 | 3.610 to 5.270 (limits of the standard) |
| annual cooling [MWh] | 4.299 | 4.382 | 4.173 | 4.333 | 4.233 | 4.117 | 2.740 to 6.030 (limits of the standard) |
| peak heating [kW] | 3.166 | 3.021 | 3.039 | 3.192 | 3.233 | 3.360 | 2.853 to 3.528 (programs ± tolerance) |
| peak cooling [kW] | 5.466 | 6.432 | 5.331 | 6.135 | 5.934 | 5.868 | 5.009 to 6.754 (programs ± tolerance) |

The overhang cuts the high summer sun and leaves the low winter sun alone, so the annual cooling drops noticeably against case 600 while the heating hardly moves. The cooling peak drops a little: it occurs in winter, when the overhang shades only part of the window.

### How the YAML describes it

The overhang is an `overhang` object of the window: its depth, the gap between the top of the window and the overhang, and how far it extends past the window on each side. Side fins (cases 630 and 930) are a `side_fins` object in the same spirit.

`spaces[0].external_boundaries.windows`:

```yaml
- surface: 12.0
  azimuth: 0.0
  tilt: wall
  construction: DOUBLE_CLEAR:001
  width: 6.0
  height: 2.0
  frame_fraction: 0.001
  overhang:
    depth: 1.0
    gap: 0.5
    width_left: 0.5
    width_right: 0.5
```

### How trano fares

| KPI | Band | Buildings |
|---|---|---|
| annual heating [MWh] | 3.610 to 5.270 | 4.475 ✓ |
| annual cooling [MWh] | 2.740 to 6.030 | 4.810 ✓ |
| peak heating [kW] | 2.853 to 3.528 | 3.216 ✓ |
| peak cooling [kW] | 5.009 to 6.754 | 6.082 ✓ |

- Buildings: 4 of 4 KPIs inside their band.
- IDEAS: the case is not supported.
- ISO 13790: the case is not supported.
- reduced order: the case is not supported.

## Case 960: Low mass zone with an unconditioned high mass sun-space

A two-zone case: the back zone of case 600 keeps its light envelope but loses its windows, and a 2 m deep unconditioned sun-space with heavy walls, a concrete slab and the 12 m2 of south glazing is attached to its south wall. The two zones share a 0.2 m concrete common wall. The back zone is conditioned between 20 and 27 degC; the sun-space floats freely.

### What the reference programs expect

| KPI | CSE | DeST | EnergyPlus | ESP-r | TRNSYS | Band |
|---|---|---|---|---|---|---|
| annual heating [MWh] | 2.522 | 2.771 | 2.689 | 2.624 | 2.860 | 2.000 to 3.400 (limits of the standard) |
| annual cooling [MWh] | 0.926 | 0.909 | 0.907 | 0.950 | 0.789 | 0.620 to 1.810 (limits of the standard) |
| peak heating [kW] | 2.132 | 2.085 | 2.259 | 2.201 | 2.300 | 1.970 to 2.415 (programs ± tolerance) |
| peak cooling [kW] | 1.377 | 1.367 | 1.480 | 1.403 | 1.338 | 1.264 to 1.554 (programs ± tolerance) |
| maximum temperature [degC] | 48.900 | 53.200 | 49.900 | 49.500 | 48.100 | 47.100 to 54.200 (programs ± tolerance) |
| minimum temperature [degC] | 8.000 | 6.700 | 5.100 | 5.000 | 4.200 | 3.200 to 9.000 (programs ± tolerance) |
| mean temperature [degC] | 28.600 | 29.500 | 27.700 | 27.700 | 26.800 | 25.800 to 30.500 (programs ± tolerance) |

The sun-space collects the solar gains, stores them in its mass and conducts part of them through the common wall into the back zone, so the back zone needs less heating than case 600 and very little cooling. The standard checks the loads of the back zone and the maximum, minimum and mean temperature of the sun-space.

### How the YAML describes it

The sun-space is a second space with `occupancy: {variant: none}` (no internal gains at all) and no emission. The common wall is an `internal_walls` entry between the two spaces; trano infers the rest of the topology from the boundaries of each space.

`spaces[1]`:

```yaml
id: SUNSPACE:001
variant: infiltration
parameters:
  floor_area: 16.0
  average_room_height: 2.7
  ach: 0.414
  linearize_emissive_power: 'false'
external_boundaries:
  external_walls:
  - surface: 21.6
    azimuth: 0.0
    tilt: wall
    construction: HEAVY_WALL:001
  - surface: 5.4
    azimuth: -1.570796
    tilt: wall
    construction: HEAVY_WALL:001
  - surface: 5.4
    azimuth: 1.570796
    tilt: wall
    construction: HEAVY_WALL:001
  - surface: 16.0
    azimuth: 0.0
    tilt: ceiling
    construction: ROOF:001
  floor_on_grounds:
  - surface: 16.0
    construction: HEAVY_FLOOR:001
    variant: outdoor_air
  windows:
  - surface: 12.0
    azimuth: 0.0
    tilt: wall
    construction: DOUBLE_CLEAR:001
    width: 6.0
    height: 2.0
    frame_fraction: 0.001
occupancy:
  variant: none
```

`internal_walls`:

```yaml
- space_1: ZONE:001
  space_2: SUNSPACE:001
  construction: COMMON_WALL:001
  surface: 21.6
```

### How trano fares

| KPI | Band | Buildings | IDEAS |
|---|---|---|---|
| annual heating [MWh] | 2.000 to 3.400 | 2.588 ✓ | 2.695 ✓ |
| annual cooling [MWh] | 0.620 to 1.810 | 0.950 ✓ | 0.950 ✓ |
| peak heating [kW] | 1.970 to 2.415 | 2.104 ✓ | 2.217 ✓ |
| peak cooling [kW] | 1.264 to 1.554 | 1.475 ✓ | 1.510 ✓ |
| maximum temperature [degC] | 47.100 to 54.200 | 48.036 ✓ | 51.309 ✓ |
| minimum temperature [degC] | 3.200 to 9.000 | 4.160 ✓ | 4.133 ✓ |
| mean temperature [degC] | 25.800 to 30.500 | 26.693 ✓ | 28.027 ✓ |

- Buildings: 7 of 7 KPIs inside their band.
- IDEAS: 7 of 7 KPIs inside their band.
- ISO 13790: the case is not supported.
- reduced order: the case is not supported.
