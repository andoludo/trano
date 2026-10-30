---
description: "Validation of the heat pumps, gas boilers, valves, pumps and ventilation units in Trano-generated Modelica models against physical KPIs."
---
# Systems validation

Do the heating and ventilation systems trano generates behave as the real ones do? Each case below puts one
system on the same 100 m2 zone (filled cavity walls, insulated roof and slab, 10 m2 of low-e double glazing
facing south, n50 = 3, occupied from 7 to 19 h, heated to 21 degC from 7 to 22 h and 16 degC at night) for one
week of the Uccle weather, and is simulated with Buildings, IDEAS and FastSmoothBuildings. The KPIs are read
from the result files by `validation/systems/kpi.py`; the band of every KPI and the physical reasoning behind
it are in `validation/systems/expected.yaml`; `pytest -m systems` fails when a KPI leaves its band. The tables
are rendered from the last results with `python -m validation.systems report --docs`.

## Heat pump

### `hp_winter`

Air-water heat pump, 4 kW at -8/45 degC, 100 l buffer, heating curve 45 to 30 degC, the coldest week. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| hours_below_setpoint [h] | 26.3 | 17.6 | 44.8 | at most 50 | pass |
| hp_buffer_below_band_hours [h] | 0 | 0.0833 | 3.17 | at most 6 | pass |
| hp_buffer_mean_c [degC] | 40.9 | 41 | 39.9 |  |  |
| hp_condenser_out_running_c [degC] | 44.7 | 44.7 | 43.6 |  |  |
| hp_cop_ratio [-] | 0.954 | 0.95 | 0.994 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.74 | 2.73 | 2.89 |  |  |
| hp_cop_table [-] | 2.87 | 2.87 | 2.91 |  |  |
| hp_electricity_kwh [kWh] | 53.3 | 55.5 | 52.1 |  |  |
| hp_heat_kwh [kWh] | 151 | 158 | 151 | 80 to 220 | pass |
| hp_pump_share [-] | 0.00289 | 0.0028 | - | at most 0.1 | pass |
| hp_run_hours [h] | 54.8 | 55.4 | 55.7 |  |  |
| hp_scop [-] | 2.83 | 2.84 | 2.9 | 2.4 to 3.6 | pass |
| hp_shortest_run_min [min] | 5 | 5 | 5 | at least 5 | pass |
| hp_shortest_stop_min [min] | 19.4 | 18.5 | 15 | at least 5 | pass |
| hp_source_running_c [degC] | -0.764 | -0.721 | -0.877 |  |  |
| hp_speed_max [-] | 0.892 | 0.98 | 0.998 | at most 1 | pass |
| hp_speed_min_running [-] | 0.3 | 0.3 | 0.0137 | at least 0.29 | pass |
| hp_starts_per_day [1/d] | 14.4 | 16.6 | 17.9 | at most 30 | pass |
| hp_supply_mean_running_c [degC] | 40.6 | 40.4 | - |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 | 42.9 | at most 45.01 | pass |
| hp_supply_setpoint_mean_c [degC] | 40.7 | 40.7 | 40.7 | 36 to 44 | pass |
| mixing_error_k [K] | 4.5 | 4.56 | 4.83 |  |  |
| mixing_max_overshoot_k [K] | 0.301 | 0.294 | -0.856 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.15 | 0.151 | 0.0638 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 | 104 |  |  |
| radiator_heat_kwh [kWh] | 150 | 156 | 149 |  |  |
| valve_hours_closed [h] | 63.5 | 63 | 63.5 |  |  |
| valve_hours_open [h] | 55.2 | 53.4 | 63.7 |  |  |
| valve_mean_opening [-] | 0.566 | 0.584 | 0.564 |  |  |
| valve_movements_per_day [1/d] | 23 | 29.3 | 22.4 |  |  |
| zone_max_c [degC] | 21.3 | 21 | 21.3 |  |  |
| zone_mean_occupied_c [degC] | 20.6 | 20.6 | 20.4 | 19.5 to 22 | pass |
| zone_min_c [degC] | 16.4 | 16.5 | 16.2 |  |  |
| zone_tracking_rmse_k [K] | 0.719 | 0.672 | 0.872 |  |  |

{!validation/figures/systems_hp_winter.html!}

### `hp_shoulder`

The same heat pump in a mild March week (part load). Week from day 75 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| hours_below_setpoint [h] | 2.3 | 2.77 | 2.5 |  |  |
| hp_buffer_below_band_hours [h] | 0.25 | 0.35 | 4.83 |  |  |
| hp_buffer_mean_c [degC] | 34.9 | 34.2 | 35.3 |  |  |
| hp_condenser_out_running_c [degC] | 39.2 | 39 | 37.3 |  |  |
| hp_cop_ratio [-] | 0.89 | 0.895 | 0.968 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 4 | 4.05 | 4.52 |  |  |
| hp_cop_table [-] | 4.49 | 4.53 | 4.67 |  |  |
| hp_electricity_kwh [kWh] | 12.1 | 13.1 | 12 |  |  |
| hp_heat_kwh [kWh] | 52.4 | 57.1 | 55.6 |  |  |
| hp_pump_share [-] | 0.00377 | 0.0038 | - |  |  |
| hp_run_hours [h] | 16 | 17.5 | 19.3 |  |  |
| hp_scop [-] | 4.33 | 4.37 | 4.63 | 2.8 to 4.9 | pass |
| hp_shortest_run_min [min] | 5.93 | 5 | 10 | at least 5 | pass |
| hp_shortest_stop_min [min] | 28.9 | 29.3 | 25 |  |  |
| hp_source_running_c [degC] | 8.73 | 8.94 | 8.22 |  |  |
| hp_speed_max [-] | 0.967 | 0.994 | 0.981 | at most 1 | pass |
| hp_speed_min_running [-] | 0.3 | 0.3 | 0.01 |  |  |
| hp_starts_per_day [1/d] | 9.43 | 10.4 | 10.4 | at most 40 | pass |
| hp_supply_mean_running_c [degC] | 34.3 | 34.1 | - |  |  |
| hp_supply_setpoint_max_c [degC] | 38 | 38 | 38 |  |  |
| hp_supply_setpoint_mean_c [degC] | 34.6 | 34.6 | 34.6 | 30 to 38 | pass |
| mixing_error_k [K] | 10.8 | 10.9 | 10.9 |  |  |
| mixing_max_overshoot_k [K] | -1.03 | -1.05 | -2.59 |  |  |
| outdoor_mean_c [degC] | 8.6 | 8.6 | 8.6 |  |  |
| pump_electricity_kwh [kWh] | 0.12 | 0.135 | 0.0308 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 92.7 | 105 | 91.9 |  |  |
| radiator_heat_kwh [kWh] | 51.7 | 56.5 | 53.9 |  |  |
| valve_hours_closed [h] | 75.3 | 63.2 | 75.6 |  |  |
| valve_hours_open [h] | 5.26 | 4.96 | 4.75 |  |  |
| valve_mean_opening [-] | 0.422 | 0.477 | 0.415 |  |  |
| valve_movements_per_day [1/d] | 24.7 | 27.3 | 30.1 |  |  |
| zone_max_c [degC] | 21.5 | 21.1 | 21.6 |  |  |
| zone_mean_occupied_c [degC] | 20.8 | 20.8 | 20.8 | 20 to 23 | pass |
| zone_min_c [degC] | 18.1 | 18 | 18 |  |  |
| zone_tracking_rmse_k [K] | 0.394 | 0.412 | 0.392 |  |  |

{!validation/figures/systems_hp_shoulder.html!}

### `hp_summer`

The same heat pump in July: above the heating limit it must not run. Week from day 196 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| hours_below_setpoint [h] | 0 | 0.417 | 0.0833 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 34.4 | 31.1 | 34.4 |  |  |
| hp_condenser_out_running_c [degC] | - | 39.2 | 43.2 |  |  |
| hp_cop_ratio [-] | - | 0.896 | 0.999 |  |  |
| hp_cop_running [-] | - | 4.82 | 4.46 |  |  |
| hp_cop_table [-] | - | 5.38 | 4.47 |  |  |
| hp_electricity_kwh [kWh] | 0 | 0.129 | 5.49e-46 | at most 1 | pass |
| hp_heat_kwh [kWh] | 0 | 0.704 | 2.25e-58 | at most 5 | pass |
| hp_pump_share [-] | - | 0.00269 | - |  |  |
| hp_run_hours [h] | 0 | 0.137 | 0.0833 | at most 0.5 | pass |
| hp_scop [-] | - | 5.47 | 4.11e-13 |  |  |
| hp_shortest_run_min [min] | - | 8.24 | 5 |  |  |
| hp_source_running_c [degC] | - | 14.8 | 12 |  |  |
| hp_speed_max [-] | 0 | 0.754 | 4.01e-58 |  |  |
| hp_speed_min_running [-] | - | 0.3 | 0 |  |  |
| hp_starts_per_day [1/d] | 0 | 0.143 | 0.143 |  |  |
| hp_supply_mean_running_c [degC] | - | 30.9 | - |  |  |
| hp_supply_setpoint_max_c [degC] | 34.4 | 34.4 | 34.4 |  |  |
| hp_supply_setpoint_mean_c [degC] | 30.4 | 30.4 | 30.4 |  |  |
| mixing_error_k [K] | 9.91 | 9.79 | 4.6 |  |  |
| mixing_max_overshoot_k [K] | -1.02 | -1.04 | -3.7 |  |  |
| outdoor_mean_c [degC] | 20 | 20 | 20 |  |  |
| pump_electricity_kwh [kWh] | 0.000266 | 0.00658 | 4.97e-05 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 0.299 | 6.93 | 0.25 |  |  |
| radiator_heat_kwh [kWh] | 0.172 | 1.39 | 0.357 |  |  |
| valve_hours_closed [h] | 167 | 160 | 167 |  |  |
| valve_hours_open [h] | 0.233 | 0.541 | 0 |  |  |
| valve_mean_opening [-] | 0.00202 | 0.0213 | 0.00136 |  |  |
| valve_movements_per_day [1/d] | 1 | 2.43 | 0.429 |  |  |
| zone_max_c [degC] | 25.2 | 25.5 | 25.1 |  |  |
| zone_mean_occupied_c [degC] | 22.9 | 22.7 | 22.8 |  |  |
| zone_min_c [degC] | 20.3 | 19.5 | 20.3 | at least 18 | pass |
| zone_tracking_rmse_k [K] | 2.18 | 2.21 | 2.12 |  |  |

{!validation/figures/systems_hp_summer.html!}

### `hp_no_buffer`

The same heat pump on a hydraulic separator instead of the 100 l buffer: more cycles. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| hours_below_setpoint [h] | 26.2 | 19.9 | 45 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0.0833 | 0.0833 |  |  |
| hp_buffer_mean_c [degC] | 41 | 41.1 | 40.3 |  |  |
| hp_condenser_out_running_c [degC] | 44.7 | 44.6 | 43.3 |  |  |
| hp_cop_ratio [-] | 0.934 | 0.929 | 0.994 | 0.8 to 1.1 | pass |
| hp_cop_running [-] | 2.68 | 2.68 | 2.91 |  |  |
| hp_cop_table [-] | 2.87 | 2.88 | 2.92 |  |  |
| hp_electricity_kwh [kWh] | 52.5 | 54.4 | 51.2 |  |  |
| hp_heat_kwh [kWh] | 149 | 155 | 148 |  |  |
| hp_pump_share [-] | 0.00306 | 0.00294 | - |  |  |
| hp_run_hours [h] | 57 | 56 | 59.3 |  |  |
| hp_scop [-] | 2.84 | 2.85 | 2.88 | 2.3 to 3.6 | pass |
| hp_shortest_run_min [min] | 5 | 5 | 5 | at least 5 | pass |
| hp_shortest_stop_min [min] | 11.7 | 11 | 10 | at least 5 | pass |
| hp_source_running_c [degC] | -0.727 | -0.663 | -0.815 |  |  |
| hp_speed_max [-] | 0.83 | 0.912 | 0.851 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 | 0.012 |  |  |
| hp_starts_per_day [1/d] | 22.3 | 26.4 | 24.6 | 10 to 80 | pass |
| hp_supply_mean_running_c [degC] | 40.7 | 40.5 | - |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 40.7 | 40.7 | 40.7 |  |  |
| mixing_error_k [K] | 4.46 | 4.54 | 4.84 |  |  |
| mixing_max_overshoot_k [K] | 0.322 | 0.316 | -0.873 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.15 | 0.151 | 0.0639 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 | 104 |  |  |
| radiator_heat_kwh [kWh] | 150 | 156 | 149 |  |  |
| valve_hours_closed [h] | 63.5 | 63 | 63.5 |  |  |
| valve_hours_open [h] | 55.9 | 53.7 | 63.8 |  |  |
| valve_mean_opening [-] | 0.566 | 0.584 | 0.564 |  |  |
| valve_movements_per_day [1/d] | 30.7 | 38.7 | 25.3 |  |  |
| zone_max_c [degC] | 21.3 | 21 | 21.3 |  |  |
| zone_mean_occupied_c [degC] | 20.6 | 20.6 | 20.4 | 19.5 to 22 | pass |
| zone_min_c [degC] | 16.4 | 16.5 | 16.2 |  |  |
| zone_tracking_rmse_k [K] | 0.66 | 0.627 | 0.87 |  |  |

{!validation/figures/systems_hp_no_buffer.html!}

### `hp_source_m10`

COP at a constant -10 degC source and 45 degC supply (EN 14511 point A-10/W45). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 25.1 | 18 |  |  |
| hp_buffer_below_band_hours [h] | 0.0833 | 0 |  |  |
| hp_buffer_mean_c [degC] | 44.6 | 45.1 |  |  |
| hp_condenser_out_running_c [degC] | 48.8 | 48.9 | 44 to 52 | pass |
| hp_cop_ratio [-] | 0.946 | 0.949 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.03 | 2.03 | 1.85 to 2.5 | pass |
| hp_cop_table [-] | 2.14 | 2.14 |  |  |
| hp_electricity_kwh [kWh] | 71.6 | 74.6 |  |  |
| hp_heat_kwh [kWh] | 152 | 159 |  |  |
| hp_pump_share [-] | 0.00201 | 0.00203 |  |  |
| hp_run_hours [h] | 47.6 | 50.2 |  |  |
| hp_scop [-] | 2.13 | 2.13 |  |  |
| hp_shortest_run_min [min] | 5 | 7.09 |  |  |
| hp_shortest_stop_min [min] | 18.9 | 9.86 |  |  |
| hp_source_running_c [degC] | -10 | -10 | -10.5 to -9.5 | pass |
| hp_speed_max [-] | 1 | 0.965 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 18.7 | 19.9 |  |  |
| hp_supply_mean_running_c [degC] | 44.5 | 44.6 |  |  |
| hp_supply_setpoint_max_c [degC] | 45 | 45 |  |  |
| hp_supply_setpoint_mean_c [degC] | 45 | 45 | 44.9 to 45.1 | pass |
| mixing_error_k [K] | 1.02 | 1 |  |  |
| mixing_max_overshoot_k [K] | 0.797 | 0.793 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.143 | 0.144 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 151 | 157 |  |  |
| valve_hours_closed [h] | 63.6 | 63.2 |  |  |
| valve_hours_open [h] | 57.6 | 52.8 |  |  |
| valve_mean_opening [-] | 0.559 | 0.574 |  |  |
| valve_movements_per_day [1/d] | 28.4 | 39.6 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.6 | 20.6 |  |  |
| zone_min_c [degC] | 16.5 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.692 | 0.662 |  |  |

{!validation/figures/systems_hp_source_m10.html!}

### `hp_source_m7`

COP at a constant -7 degC source and 45 degC supply (EN 14511 point A-7/W45). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 25.4 | 18.9 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 44.6 | 44.8 |  |  |
| hp_condenser_out_running_c [degC] | 48.9 | 49 | 44 to 52 | pass |
| hp_cop_ratio [-] | 0.952 | 0.951 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.14 | 2.14 | 1.95 to 2.65 | pass |
| hp_cop_table [-] | 2.25 | 2.25 |  |  |
| hp_electricity_kwh [kWh] | 68.2 | 70.9 |  |  |
| hp_heat_kwh [kWh] | 152 | 159 |  |  |
| hp_pump_share [-] | 0.002 | 0.00202 |  |  |
| hp_run_hours [h] | 49.8 | 52.5 |  |  |
| hp_scop [-] | 2.24 | 2.24 |  |  |
| hp_shortest_run_min [min] | 5 | 5 |  |  |
| hp_shortest_stop_min [min] | 19 | 9.73 |  |  |
| hp_source_running_c [degC] | -7 | -7 | -7.5 to -6.5 | pass |
| hp_speed_max [-] | 0.985 | 0.978 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 17.7 | 19.1 |  |  |
| hp_supply_mean_running_c [degC] | 44.5 | 44.5 |  |  |
| hp_supply_setpoint_max_c [degC] | 45 | 45 |  |  |
| hp_supply_setpoint_mean_c [degC] | 45 | 45 | 44.9 to 45.1 | pass |
| mixing_error_k [K] | 1.03 | 1.02 |  |  |
| mixing_max_overshoot_k [K] | 0.897 | 0.773 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.143 | 0.144 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 150 | 156 |  |  |
| valve_hours_closed [h] | 63.6 | 63.2 |  |  |
| valve_hours_open [h] | 57.6 | 53.4 |  |  |
| valve_mean_opening [-] | 0.559 | 0.575 |  |  |
| valve_movements_per_day [1/d] | 29 | 39.3 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.6 | 20.6 |  |  |
| zone_min_c [degC] | 16.4 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.707 | 0.671 |  |  |

{!validation/figures/systems_hp_source_m7.html!}

### `hp_source_2`

COP at a constant 2 degC source and 45 degC supply (EN 14511 point A2/W45). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 27.3 | 19.5 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 45 | 44.8 |  |  |
| hp_condenser_out_running_c [degC] | 49.2 | 49.2 | 44 to 52 | pass |
| hp_cop_ratio [-] | 0.957 | 0.957 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.82 | 2.82 | 2.55 to 3.45 | pass |
| hp_cop_table [-] | 2.95 | 2.95 |  |  |
| hp_electricity_kwh [kWh] | 52 | 53.9 |  |  |
| hp_heat_kwh [kWh] | 152 | 158 |  |  |
| hp_pump_share [-] | 0.00252 | 0.00253 |  |  |
| hp_run_hours [h] | 52.4 | 54.8 |  |  |
| hp_scop [-] | 2.93 | 2.93 |  |  |
| hp_shortest_run_min [min] | 9.78 | 5 |  |  |
| hp_shortest_stop_min [min] | 18.7 | 9.62 |  |  |
| hp_source_running_c [degC] | 2 | 2 | 1.5 to 2.5 | pass |
| hp_speed_max [-] | 0.97 | 0.943 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 16.7 | 18 |  |  |
| hp_supply_mean_running_c [degC] | 44.6 | 44.6 |  |  |
| hp_supply_setpoint_max_c [degC] | 45 | 45 |  |  |
| hp_supply_setpoint_mean_c [degC] | 45 | 45 | 44.9 to 45.1 | pass |
| mixing_error_k [K] | 1.01 | 1 |  |  |
| mixing_max_overshoot_k [K] | 0.871 | 0.761 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.143 | 0.143 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 150 | 156 |  |  |
| valve_hours_closed [h] | 63.6 | 63.2 |  |  |
| valve_hours_open [h] | 58.6 | 53.8 |  |  |
| valve_mean_opening [-] | 0.559 | 0.575 |  |  |
| valve_movements_per_day [1/d] | 25.9 | 35 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.5 | 20.6 |  |  |
| zone_min_c [degC] | 16.4 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.728 | 0.697 |  |  |

{!validation/figures/systems_hp_source_2.html!}

### `hp_source_7`

COP at a constant 7 degC source and 45 degC supply (EN 14511 point A7/W45). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 49.2 | 52.3 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 45.8 | 46.1 |  |  |
| hp_condenser_out_running_c [degC] | 49.7 | 49.7 | 44 to 52 | pass |
| hp_cop_ratio [-] | 0.981 | 0.982 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 3.36 | 3.37 | 3 to 4.1 | pass |
| hp_cop_table [-] | 3.43 | 3.43 |  |  |
| hp_electricity_kwh [kWh] | 40.3 | 42.4 |  |  |
| hp_heat_kwh [kWh] | 137 | 144 |  |  |
| hp_pump_share [-] | 0.00357 | 0.00365 |  |  |
| hp_run_hours [h] | 74.4 | 80.1 |  |  |
| hp_scop [-] | 3.41 | 3.41 |  |  |
| hp_shortest_run_min [min] | 11.7 | 8.42 |  |  |
| hp_shortest_stop_min [min] | 19.6 | 9.28 |  |  |
| hp_source_running_c [degC] | 7 | 7 | 6.5 to 7.5 | pass |
| hp_speed_max [-] | 0.801 | 0.784 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 7.57 | 7.71 |  |  |
| hp_supply_mean_running_c [degC] | 45.8 | 45.9 |  |  |
| hp_supply_setpoint_max_c [degC] | 45 | 45 |  |  |
| hp_supply_setpoint_mean_c [degC] | 45 | 45 | 44.9 to 45.1 | pass |
| mixing_error_k [K] | 0.699 | 0.661 |  |  |
| mixing_max_overshoot_k [K] | 0.774 | 0.713 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.138 | 0.139 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 135 | 142 |  |  |
| valve_hours_closed [h] | 63.6 | 63.2 |  |  |
| valve_hours_open [h] | 61.6 | 65.9 |  |  |
| valve_mean_opening [-] | 0.561 | 0.582 |  |  |
| valve_movements_per_day [1/d] | 19.9 | 19.6 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.3 | 20.4 |  |  |
| zone_min_c [degC] | 16 | 16.1 |  |  |
| zone_tracking_rmse_k [K] | 1.02 | 0.987 |  |  |

{!validation/figures/systems_hp_source_7.html!}

### `hp_source_12`

COP at a constant 12 degC source and 45 degC supply (EN 14511 point A12/W45). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 55 | 54.5 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 45.9 | 46 |  |  |
| hp_condenser_out_running_c [degC] | 49.8 | 49.9 | 44 to 52 | pass |
| hp_cop_ratio [-] | 0.991 | 0.99 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 3.9 | 3.89 | 3.45 to 4.7 | pass |
| hp_cop_table [-] | 3.93 | 3.93 |  |  |
| hp_electricity_kwh [kWh] | 36.5 | 38.3 |  |  |
| hp_heat_kwh [kWh] | 143 | 150 |  |  |
| hp_pump_share [-] | 0.0036 | 0.00365 |  |  |
| hp_run_hours [h] | 86.7 | 92.1 |  |  |
| hp_scop [-] | 3.91 | 3.91 |  |  |
| hp_shortest_run_min [min] | 11.7 | 19.8 |  |  |
| hp_shortest_stop_min [min] | 27.1 | 8.92 |  |  |
| hp_source_running_c [degC] | 12 | 12 | 11.5 to 12.5 | pass |
| hp_speed_max [-] | 0.683 | 0.696 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 3.43 | 3.57 |  |  |
| hp_supply_mean_running_c [degC] | 45.8 | 45.9 |  |  |
| hp_supply_setpoint_max_c [degC] | 45 | 45 |  |  |
| hp_supply_setpoint_mean_c [degC] | 45 | 45 | 44.9 to 45.1 | pass |
| mixing_error_k [K] | 0.466 | 0.448 |  |  |
| mixing_max_overshoot_k [K] | 0.656 | 0.652 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.14 | 0.14 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 140 | 147 |  |  |
| valve_hours_closed [h] | 63.6 | 63.1 |  |  |
| valve_hours_open [h] | 67 | 70.9 |  |  |
| valve_mean_opening [-] | 0.568 | 0.588 |  |  |
| valve_movements_per_day [1/d] | 7.57 | 7.57 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.4 | 20.4 |  |  |
| zone_min_c [degC] | 16.1 | 16.2 |  |  |
| zone_tracking_rmse_k [K] | 1.02 | 0.997 |  |  |

{!validation/figures/systems_hp_source_12.html!}

### `hp_table_winter`

The air-water heat pump given as a table (heat_pump_data: table) holding the Vitocal 251-A08 data: it must reproduce hp_winter. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 20.4 | 11.8 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0.0833 |  |  |
| hp_buffer_mean_c [degC] | 41.8 | 41.6 |  |  |
| hp_condenser_out_running_c [degC] | 45.3 | 45.2 |  |  |
| hp_cop_ratio [-] | 0.983 | 0.981 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.8 | 2.8 |  |  |
| hp_cop_table [-] | 2.85 | 2.85 |  |  |
| hp_electricity_kwh [kWh] | 54.5 | 56.5 |  |  |
| hp_heat_kwh [kWh] | 153 | 160 | 80 to 220 | pass |
| hp_pump_share [-] | 0.00281 | 0.00283 |  |  |
| hp_run_hours [h] | 81.4 | 85.1 |  |  |
| hp_scop [-] | 2.82 | 2.82 | 2.4 to 3.6 | pass |
| hp_shortest_run_min [min] | 14.9 | 5 |  |  |
| hp_shortest_stop_min [min] | 25.9 | 22.1 |  |  |
| hp_source_running_c [degC] | -0.755 | -0.705 |  |  |
| hp_speed_max [-] | 0.678 | 0.766 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 4.57 | 5.43 |  |  |
| hp_supply_mean_running_c [degC] | 41.5 | 41.4 |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 40.7 | 40.7 |  |  |
| mixing_error_k [K] | 3.95 | 3.94 |  |  |
| mixing_max_overshoot_k [K] | -0.21 | -0.34 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.149 | 0.15 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 152 | 158 |  |  |
| valve_hours_closed [h] | 63.5 | 63 |  |  |
| valve_hours_open [h] | 45.4 | 30 |  |  |
| valve_mean_opening [-] | 0.562 | 0.577 |  |  |
| valve_movements_per_day [1/d] | 13 | 14.4 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.6 | 20.6 | 19.5 to 22 | pass |
| zone_min_c [degC] | 16.5 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.762 | 0.758 |  |  |

{!validation/figures/systems_hp_table_winter.html!}

### `hp_carnot_winter`

The air-water heat pump as a Carnot model (effectiveness 0.45, frosting of the evaporator) in the coldest week. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 34.6 | 22.6 |  |  |
| hp_buffer_below_band_hours [h] | 0.0833 | 0 |  |  |
| hp_buffer_mean_c [degC] | 40.5 | 40.7 |  |  |
| hp_condenser_out_running_c [degC] | 45.6 | 45.5 |  |  |
| hp_cop_ratio [-] | 0.743 | 0.743 |  |  |
| hp_cop_running [-] | 2.1 | 2.11 |  |  |
| hp_cop_table [-] | 2.83 | 2.84 |  |  |
| hp_electricity_kwh [kWh] | 67.9 | 71.2 |  |  |
| hp_heat_kwh [kWh] | 149 | 156 | 80 to 220 | pass |
| hp_pump_share [-] | 0.00197 | 0.00196 |  |  |
| hp_run_hours [h] | 42.3 | 44.3 |  |  |
| hp_scop [-] | 2.19 | 2.19 | 2 to 3.4 | pass |
| hp_shortest_run_min [min] | 5 | 5 | at least 5 | pass |
| hp_shortest_stop_min [min] | 18.2 | 18.3 |  |  |
| hp_source_running_c [degC] | -0.802 | -0.752 |  |  |
| hp_speed_max [-] | 1 | 1 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 | at least 0.29 | pass |
| hp_starts_per_day [1/d] | 19.4 | 20.7 |  |  |
| hp_supply_mean_running_c [degC] | 40.3 | 40.3 |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 40.7 | 40.7 |  |  |
| mixing_error_k [K] | 4.6 | 4.59 |  |  |
| mixing_max_overshoot_k [K] | 0.334 | 0.347 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.151 | 0.152 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 148 | 155 |  |  |
| valve_hours_closed [h] | 63.5 | 63 |  |  |
| valve_hours_open [h] | 59.6 | 55.8 |  |  |
| valve_mean_opening [-] | 0.567 | 0.585 |  |  |
| valve_movements_per_day [1/d] | 21.9 | 28.7 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.5 | 20.6 | 19.5 to 22 | pass |
| zone_min_c [degC] | 16.4 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.746 | 0.705 |  |  |

{!validation/figures/systems_hp_carnot_winter.html!}

### `gshp_winter`

Ground-source heat pump (Carnot, effectiveness 0.45) on a ground loop at 10 degC (3 K annual swing) in the coldest week. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 34.3 | 22.2 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 40.3 | 40.8 |  |  |
| hp_condenser_out_running_c [degC] | 45.7 | 45.6 |  |  |
| hp_cop_ratio [-] | 0.8 | 0.802 |  |  |
| hp_cop_running [-] | 2.97 | 2.98 | 2.8 to 4.8 | pass |
| hp_cop_table [-] | 3.71 | 3.72 |  |  |
| hp_electricity_kwh [kWh] | 47.9 | 50.2 |  |  |
| hp_heat_kwh [kWh] | 149 | 156 | 80 to 220 | pass |
| hp_pump_share [-] | 0.00278 | 0.00281 |  |  |
| hp_run_hours [h] | 41.8 | 44 |  |  |
| hp_scop [-] | 3.1 | 3.11 | 3 to 4.6 | pass |
| hp_shortest_run_min [min] | 5 | 5 |  |  |
| hp_shortest_stop_min [min] | 18.4 | 18.3 |  |  |
| hp_source_running_c [degC] | 7.25 | 7.26 | 6.5 to 8.5 | pass |
| hp_speed_max [-] | 1 | 1 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 19.7 | 21 |  |  |
| hp_supply_mean_running_c [degC] | 40.3 | 40.3 |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 40.7 | 40.7 |  |  |
| mixing_error_k [K] | 4.59 | 4.58 |  |  |
| mixing_max_overshoot_k [K] | 0.346 | 0.349 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.151 | 0.152 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 147 | 155 |  |  |
| valve_hours_closed [h] | 63.5 | 63 |  |  |
| valve_hours_open [h] | 59.2 | 56.3 |  |  |
| valve_mean_opening [-] | 0.567 | 0.585 |  |  |
| valve_movements_per_day [1/d] | 21.3 | 28.1 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.5 | 20.6 | 19.5 to 22 | pass |
| zone_min_c [degC] | 16.4 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.74 | 0.696 |  |  |

{!validation/figures/systems_gshp_winter.html!}

### `wwhp_winter`

Water-water heat pump (Carnot, effectiveness 0.45) on a water loop at 12 degC in the coldest week. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 34.6 | 22 |  |  |
| hp_buffer_below_band_hours [h] | 0.0833 | 0 |  |  |
| hp_buffer_mean_c [degC] | 40.5 | 40.7 |  |  |
| hp_condenser_out_running_c [degC] | 45.6 | 45.6 |  |  |
| hp_cop_ratio [-] | 0.783 | 0.783 |  |  |
| hp_cop_running [-] | 3.33 | 3.33 |  |  |
| hp_cop_table [-] | 4.25 | 4.25 |  |  |
| hp_electricity_kwh [kWh] | 42.9 | 45 |  |  |
| hp_heat_kwh [kWh] | 149 | 156 | 80 to 220 | pass |
| hp_pump_share [-] | 0.00311 | 0.00309 |  |  |
| hp_run_hours [h] | 42 | 44.1 |  |  |
| hp_scop [-] | 3.46 | 3.46 | 3.2 to 5 | pass |
| hp_shortest_run_min [min] | 5 | 5 |  |  |
| hp_shortest_stop_min [min] | 18.5 | 18.3 |  |  |
| hp_source_running_c [degC] | 12 | 12 | 11.5 to 12.5 | pass |
| hp_speed_max [-] | 1 | 1 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 19.4 | 20.7 |  |  |
| hp_supply_mean_running_c [degC] | 40.3 | 40.3 |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 40.7 | 40.7 |  |  |
| mixing_error_k [K] | 4.59 | 4.58 |  |  |
| mixing_max_overshoot_k [K] | 0.335 | 0.347 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.151 | 0.152 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 148 | 155 |  |  |
| valve_hours_closed [h] | 63.5 | 63 |  |  |
| valve_hours_open [h] | 58.9 | 56.1 |  |  |
| valve_mean_opening [-] | 0.567 | 0.585 |  |  |
| valve_movements_per_day [1/d] | 21.9 | 29 |  |  |
| zone_max_c [degC] | 21.3 | 21 |  |  |
| zone_mean_occupied_c [degC] | 20.5 | 20.6 | 19.5 to 22 | pass |
| zone_min_c [degC] | 16.4 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.746 | 0.702 |  |  |

{!validation/figures/systems_wwhp_winter.html!}

### `hp_supply_35`

COP at a constant 7 degC source and 35 degC supply (EN 14511 point A7/W35). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 79.2 | 83.7 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 35 | 35.2 |  |  |
| hp_condenser_out_running_c [degC] | 39.4 | 39.4 | 34 to 42 | pass |
| hp_cop_ratio [-] | 0.947 | 0.95 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 4.07 | 4.08 | 3.8 to 5.15 | pass |
| hp_cop_table [-] | 4.29 | 4.29 |  |  |
| hp_electricity_kwh [kWh] | 25.1 | 26.5 |  |  |
| hp_heat_kwh [kWh] | 107 | 113 |  |  |
| hp_pump_share [-] | 0.00349 | 0.00348 |  |  |
| hp_run_hours [h] | 42.5 | 44.7 |  |  |
| hp_scop [-] | 4.26 | 4.25 |  |  |
| hp_shortest_run_min [min] | 9.58 | 15.5 |  |  |
| hp_shortest_stop_min [min] | 25.7 | 30.2 |  |  |
| hp_source_running_c [degC] | 7 | 7 |  |  |
| hp_speed_max [-] | 0.74 | 0.746 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 15.3 | 15.7 |  |  |
| hp_supply_mean_running_c [degC] | 34.7 | 34.7 |  |  |
| hp_supply_setpoint_max_c [degC] | 35 | 35 |  |  |
| hp_supply_setpoint_mean_c [degC] | 35 | 35 | 34.9 to 35.1 | pass |
| mixing_error_k [K] | 0.923 | 0.928 |  |  |
| mixing_max_overshoot_k [K] | 0.874 | 0.864 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.165 | 0.166 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 119 | 119 |  |  |
| radiator_heat_kwh [kWh] | 105 | 111 |  |  |
| valve_hours_closed [h] | 47.7 | 47.1 |  |  |
| valve_hours_open [h] | 86.9 | 93.4 |  |  |
| valve_mean_opening [-] | 0.645 | 0.663 |  |  |
| valve_movements_per_day [1/d] | 11.1 | 8 |  |  |
| zone_max_c [degC] | 21.1 | 21 |  |  |
| zone_mean_occupied_c [degC] | 19.3 | 19.3 |  |  |
| zone_min_c [degC] | 15.8 | 15.9 |  |  |
| zone_tracking_rmse_k [K] | 2.01 | 1.97 |  |  |

{!validation/figures/systems_hp_supply_35.html!}

### `hp_supply_55`

COP at a constant 7 degC source and 55 degC supply (EN 14511 point A7/W55). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 2.57 | 2.82 |  |  |
| hp_buffer_below_band_hours [h] | 0.0667 | 0.0667 |  |  |
| hp_buffer_mean_c [degC] | 55.5 | 55.8 |  |  |
| hp_condenser_out_running_c [degC] | 59.8 | 59.7 | 54 to 62 | pass |
| hp_cop_ratio [-] | 0.987 | 0.986 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.84 | 2.84 | 2.5 to 3.45 | pass |
| hp_cop_table [-] | 2.88 | 2.88 |  |  |
| hp_electricity_kwh [kWh] | 56.3 | 57.6 |  |  |
| hp_heat_kwh [kWh] | 161 | 165 |  |  |
| hp_pump_share [-] | 0.00277 | 0.00285 |  |  |
| hp_run_hours [h] | 81.2 | 85.6 |  |  |
| hp_scop [-] | 2.86 | 2.86 |  |  |
| hp_shortest_run_min [min] | 9.24 | 6.74 |  |  |
| hp_shortest_stop_min [min] | 9.03 | 5 |  |  |
| hp_source_running_c [degC] | 7 | 7 |  |  |
| hp_speed_max [-] | 0.972 | 0.961 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 5 | 5.71 |  |  |
| hp_supply_mean_running_c [degC] | 55.8 | 55.9 |  |  |
| hp_supply_setpoint_max_c [degC] | 55 | 55 |  |  |
| hp_supply_setpoint_mean_c [degC] | 55 | 55 | 54.9 to 55.1 | pass |
| mixing_error_k [K] | 0.612 | 0.609 |  |  |
| mixing_max_overshoot_k [K] | 0.621 | 0.63 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.135 | 0.134 |  |  |
| pump_hours_without_demand [h] | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 |  |  |
| radiator_heat_kwh [kWh] | 158 | 162 |  |  |
| valve_hours_closed [h] | 63.7 | 63.3 |  |  |
| valve_hours_open [h] | 4.11 | 4.14 |  |  |
| valve_mean_opening [-] | 0.508 | 0.52 |  |  |
| valve_movements_per_day [1/d] | 14.1 | 13.6 |  |  |
| zone_max_c [degC] | 21.5 | 21.2 |  |  |
| zone_mean_occupied_c [degC] | 20.7 | 20.7 |  |  |
| zone_min_c [degC] | 16.6 | 16.6 |  |  |
| zone_tracking_rmse_k [K] | 0.664 | 0.685 |  |  |

{!validation/figures/systems_hp_supply_55.html!}

## Boiler

### `boiler_storage_winter`

Gas boiler, 6 kW, 200 l storage tank, 70 degC set point, the coldest week. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.877 | 0.877 | 0.846 | 0.8 to 0.91 | pass |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 31.1 | 31.9 | 96.5 |  |  |
| boiler_fuel_kwh [kWh] | 207 | 212 | 190 |  |  |
| boiler_gas_m3 [m3] | 17.8 | 18.2 | - |  |  |
| boiler_heat_kwh [kWh] | 182 | 186 | 161 | 80 to 250 | pass |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - | 11.6 to 11.75 | pass |
| boiler_shortest_firing_min [min] | 65.6 | 58.8 | 10 |  |  |
| boiler_starts_per_day [1/d] | 3.29 | 3.43 | 4.57 | 1 to 40 | pass |
| boiler_water_firing_c [degC] | 71.4 | 71.5 | 69.1 |  |  |
| boiler_water_max_c [degC] | 87.6 | 87.6 | 70 | at most 95 | pass |
| hours_below_setpoint [h] | 15.5 | 11.3 | 4.08 |  |  |
| mixing_error_k [K] | 1.59 | 1.38 | 1.39 |  |  |
| mixing_max_overshoot_k [K] | 0.749 | 0.822 | -0.369 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.131 | 0.133 | 0.0327 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 105 | 98.3 |  |  |
| radiator_heat_kwh [kWh] | 152 | 156 | 161 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 | - |  |  |
| tank_top_max_c [degC] | 84.6 | 84.5 | - | 70 to 92 | pass |
| tank_top_min_c [degC] | 62.8 | 62.6 | - |  |  |
| valve_hours_closed [h] | 63.8 | 63.2 | 64.6 |  |  |
| valve_hours_open [h] | 36.1 | 27.4 | 6.83 |  |  |
| valve_mean_opening [-] | 0.521 | 0.533 | 0.473 |  |  |
| valve_movements_per_day [1/d] | 17.9 | 16.4 | 23.9 |  |  |
| zone_max_c [degC] | 21.4 | 21.1 | 21.6 |  |  |
| zone_mean_occupied_c [degC] | 20.7 | 20.7 | 20.7 | 19.5 to 22.5 | pass |
| zone_min_c [degC] | 16.4 | 16.4 | 16.4 |  |  |
| zone_tracking_rmse_k [K] | 0.667 | 0.667 | 0.648 |  |  |

{!validation/figures/systems_boiler_storage_winter.html!}

### `boiler_no_storage_winter`

Gas boiler without storage, 6 kW, 70 degC supply set point, the coldest week. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.875 | 0.875 | 0.846 | 0.84 to 0.91 | pass |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 95.9 | 104 | 96.5 |  |  |
| boiler_fuel_kwh [kWh] | 184 | 188 | 190 |  |  |
| boiler_gas_m3 [m3] | 15.7 | 16.1 | - |  |  |
| boiler_heat_kwh [kWh] | 161 | 164 | 161 | 80 to 250 | pass |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - | 11.6 to 11.75 | pass |
| boiler_shortest_firing_min [min] | 10 | 13.6 | 10 |  |  |
| boiler_starts_per_day [1/d] | 1.86 | 1.71 | 4.57 |  |  |
| boiler_water_firing_c [degC] | 69.4 | 69.5 | 69.1 |  |  |
| boiler_water_max_c [degC] | 70 | 70 | 70 | at most 85 | pass |
| hours_below_setpoint [h] | 3.33 | 3.58 | 4.08 |  |  |
| mixing_error_k [K] | 0.855 | 0.835 | 1.39 |  |  |
| mixing_max_overshoot_k [K] | -0.0293 | -0.0287 | -0.369 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.131 | 0.133 | 0.0327 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 104 | 104 | 98.3 |  |  |
| radiator_heat_kwh [kWh] | 156 | 159 | 161 |  |  |
| valve_hours_closed [h] | 64.2 | 63.6 | 64.6 |  |  |
| valve_hours_open [h] | 4.45 | 4.75 | 6.83 |  |  |
| valve_mean_opening [-] | 0.467 | 0.479 | 0.473 |  |  |
| valve_movements_per_day [1/d] | 11.6 | 13.6 | 23.9 |  |  |
| zone_max_c [degC] | 21.6 | 21.4 | 21.6 |  |  |
| zone_mean_occupied_c [degC] | 20.7 | 20.7 | 20.7 | 19.5 to 22.5 | pass |
| zone_min_c [degC] | 16.5 | 16.5 | 16.4 |  |  |
| zone_tracking_rmse_k [K] | 0.629 | 0.656 | 0.648 |  |  |

{!validation/figures/systems_boiler_no_storage_winter.html!}

### `boiler_condensing_winter`

Condensing boiler without storage, efficiency 0.97 at 40 degC to 0.90 at 70 degC, radiators 45/35. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.942 | 0.943 | 0.925 | 0.9 to 0.99 | pass |
| boiler_eta_firing [-] | 0.958 | 0.958 | 0.958 | 0.93 to 0.99 | pass |
| boiler_firing_hours [h] | 95.1 | 105 | 97.7 |  |  |
| boiler_fuel_kwh [kWh] | 163 | 169 | 147 |  |  |
| boiler_gas_m3 [m3] | 14 | 14.5 | - |  |  |
| boiler_heat_kwh [kWh] | 154 | 159 | 136 |  |  |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - |  |  |
| boiler_shortest_firing_min [min] | 9.18 | 22.9 | 5 |  |  |
| boiler_starts_per_day [1/d] | 1.57 | 1.14 | 4.29 |  |  |
| boiler_water_firing_c [degC] | 44.6 | 44.7 | 44.6 |  |  |
| boiler_water_max_c [degC] | 45 | 45 | 45 | at most 60 | pass |
| hours_below_setpoint [h] | 21.8 | 11.9 | 62 |  |  |
| mixing_error_k [K] | 0.489 | 0.488 | 1.03 |  |  |
| mixing_max_overshoot_k [K] | -0.0288 | -0.0809 | -0.76 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.147 | 0.147 | 0.0478 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0.0833 |  |  |
| pump_run_hours [h] | 104 | 105 | 104 |  |  |
| radiator_heat_kwh [kWh] | 151 | 157 | 136 |  |  |
| valve_hours_closed [h] | 63.4 | 63.1 | 61.2 |  |  |
| valve_hours_open [h] | 48.2 | 32.2 | 68.8 |  |  |
| valve_mean_opening [-] | 0.555 | 0.568 | 0.567 |  |  |
| valve_movements_per_day [1/d] | 6.14 | 6.43 | 10.1 |  |  |
| zone_max_c [degC] | 21.3 | 21 | 21.2 |  |  |
| zone_mean_occupied_c [degC] | 20.6 | 20.6 | 20.1 | 19.5 to 22.5 | pass |
| zone_min_c [degC] | 16.4 | 16.4 | 15.9 |  |  |
| zone_tracking_rmse_k [K] | 0.71 | 0.706 | 1.21 |  |  |

{!validation/figures/systems_boiler_condensing_winter.html!}

### `boiler_storage_shoulder`

The boiler with storage in a mild March week (part load). Week from day 75 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.861 | 0.862 | 0.756 | 0.7 to 0.91 | pass |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 13.4 | 14.8 | 54.2 |  |  |
| boiler_fuel_kwh [kWh] | 89 | 98.5 | 71.3 |  |  |
| boiler_gas_m3 [m3] | 7.63 | 8.44 | - |  |  |
| boiler_heat_kwh [kWh] | 76.6 | 84.9 | 53.9 |  |  |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - |  |  |
| boiler_shortest_firing_min [min] | 58.7 | 68.6 | 5 |  |  |
| boiler_starts_per_day [1/d] | 1.43 | 1.57 | 20.6 | at most 40 | pass |
| boiler_water_firing_c [degC] | 68.5 | 68.1 | 69.5 |  |  |
| boiler_water_max_c [degC] | 87.6 | 87.6 | 70 |  |  |
| hours_below_setpoint [h] | 2.13 | 2.41 | 2.08 |  |  |
| mixing_error_k [K] | 1.63 | 1.66 | 1.04 |  |  |
| mixing_max_overshoot_k [K] | 0.631 | 0.638 | -0.882 |  |  |
| outdoor_mean_c [degC] | 8.6 | 8.6 | 8.6 |  |  |
| pump_electricity_kwh [kWh] | 0.109 | 0.127 | 0.0118 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 91.2 | 103 | 82.4 |  |  |
| radiator_heat_kwh [kWh] | 51.7 | 56.4 | 54.7 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 | - |  |  |
| tank_top_max_c [degC] | 84.5 | 84.5 | - |  |  |
| tank_top_min_c [degC] | 60 | 60.2 | - |  |  |
| valve_hours_closed [h] | 76.7 | 64.7 | 78.6 |  |  |
| valve_hours_open [h] | 2.99 | 3.43 | 2.17 |  |  |
| valve_mean_opening [-] | 0.31 | 0.35 | 0.241 |  |  |
| valve_movements_per_day [1/d] | 16.7 | 18.7 | 51.6 |  |  |
| zone_max_c [degC] | 21.6 | 21.3 | 21.7 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.8 | 20.9 | 20 to 23.5 | pass |
| zone_min_c [degC] | 18 | 18 | 18 |  |  |
| zone_tracking_rmse_k [K] | 0.37 | 0.39 | 0.379 |  |  |

{!validation/figures/systems_boiler_storage_shoulder.html!}

### `boiler_storage_summer`

The boiler with storage in July: no heat demand, only the losses of the tank. Week from day 196 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.838 | 0.84 | 0.00029 |  |  |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 5 | 5.21 | 0.333 |  |  |
| boiler_fuel_kwh [kWh] | 33.3 | 34.7 | 2.85 | at most 45 | pass |
| boiler_gas_m3 [m3] | 2.86 | 2.98 | - |  |  |
| boiler_heat_kwh [kWh] | 27.9 | 29.2 | 0.000829 | at most 40 | pass |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - |  |  |
| boiler_shortest_firing_min [min] | 36.6 | 36.6 | 5 |  |  |
| boiler_starts_per_day [1/d] | 0.714 | 0.714 | 0.571 |  |  |
| boiler_water_firing_c [degC] | 68.2 | 67.6 | 53.4 |  |  |
| boiler_water_max_c [degC] | 87.6 | 87.6 | 70 |  |  |
| hours_below_setpoint [h] | 0 | 0.417 | 0.0833 |  |  |
| mixing_error_k [K] | 15.9 | 3.74 | 13.5 |  |  |
| mixing_max_overshoot_k [K] | -0.468 | 0.000718 | -0.899 |  |  |
| outdoor_mean_c [degC] | 20 | 20 | 20 |  |  |
| pump_electricity_kwh [kWh] | 0.000426 | 0.00453 | 0.000146 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 0.413 | 4.43 | 0.417 |  |  |
| radiator_heat_kwh [kWh] | 0.438 | 2.05 | 0.48 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 | - |  |  |
| tank_top_max_c [degC] | 84.5 | 84.5 | - |  |  |
| tank_top_min_c [degC] | 67.2 | 67.3 | - |  |  |
| valve_hours_closed [h] | 167 | 162 | 167 |  |  |
| valve_hours_open [h] | 0.24 | 0.537 | 0.0833 |  |  |
| valve_mean_opening [-] | 0.00243 | 0.0135 | 0.00238 |  |  |
| valve_movements_per_day [1/d] | 1 | 2.71 | 1 |  |  |
| zone_max_c [degC] | 25.2 | 25.6 | 25.1 |  |  |
| zone_mean_occupied_c [degC] | 22.9 | 22.8 | 22.8 |  |  |
| zone_min_c [degC] | 20.3 | 19.4 | 20.3 | at least 18 | pass |
| zone_tracking_rmse_k [K] | 2.22 | 2.28 | 2.14 |  |  |

{!validation/figures/systems_boiler_storage_summer.html!}

## Hydronics

### `valve_p`

Radiator valve under a proportional controller on the zone temperature (PController). Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.877 | 0.877 | 0.846 |  |  |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 31.1 | 31.9 | 96.5 |  |  |
| boiler_fuel_kwh [kWh] | 207 | 212 | 190 |  |  |
| boiler_gas_m3 [m3] | 17.8 | 18.2 | - |  |  |
| boiler_heat_kwh [kWh] | 182 | 186 | 161 |  |  |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - |  |  |
| boiler_shortest_firing_min [min] | 65.6 | 58.8 | 10 |  |  |
| boiler_starts_per_day [1/d] | 3.29 | 3.43 | 4.57 |  |  |
| boiler_water_firing_c [degC] | 71.4 | 71.5 | 69.1 |  |  |
| boiler_water_max_c [degC] | 87.6 | 87.6 | 70 |  |  |
| hours_below_setpoint [h] | 15.5 | 11.3 | 4.08 |  |  |
| mixing_error_k [K] | 1.59 | 1.38 | 1.39 | at most 3 | pass |
| mixing_max_overshoot_k [K] | 0.749 | 0.822 | -0.369 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.131 | 0.133 | 0.0327 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 | at most 1 | pass |
| pump_run_hours [h] | 104 | 105 | 98.3 |  |  |
| radiator_heat_kwh [kWh] | 152 | 156 | 161 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 | - |  |  |
| tank_top_max_c [degC] | 84.6 | 84.5 | - |  |  |
| tank_top_min_c [degC] | 62.8 | 62.6 | - |  |  |
| valve_hours_closed [h] | 63.8 | 63.2 | 64.6 | at least 20 | pass |
| valve_hours_open [h] | 36.1 | 27.4 | 6.83 |  |  |
| valve_mean_opening [-] | 0.521 | 0.533 | 0.473 |  |  |
| valve_movements_per_day [1/d] | 17.9 | 16.4 | 23.9 | at most 60 | pass |
| zone_max_c [degC] | 21.4 | 21.1 | 21.6 |  |  |
| zone_mean_occupied_c [degC] | 20.7 | 20.7 | 20.7 | 19.5 to 22.5 | pass |
| zone_min_c [degC] | 16.4 | 16.4 | 16.4 |  |  |
| zone_tracking_rmse_k [K] | 0.667 | 0.667 | 0.648 | at most 1.5 | pass |

{!validation/figures/systems_valve_p.html!}

### `valve_pi`

Radiator valve under the Guideline 36 heating loop (the default emission control). Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.876 | 0.875 | 0.852 |  |  |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 29.9 | 30 | 99.8 |  |  |
| boiler_fuel_kwh [kWh] | 199 | 200 | 194 |  |  |
| boiler_gas_m3 [m3] | 17.1 | 17.1 | - |  |  |
| boiler_heat_kwh [kWh] | 174 | 175 | 165 |  |  |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - |  |  |
| boiler_shortest_firing_min [min] | 70.4 | 61.1 | 160 |  |  |
| boiler_starts_per_day [1/d] | 3 | 3.14 | 1.14 |  |  |
| boiler_water_firing_c [degC] | 71.8 | 71.9 | 69.7 |  |  |
| boiler_water_max_c [degC] | 87.7 | 87.7 | 70 |  |  |
| hours_below_setpoint [h] | 38.6 | 36 | 7.83 |  |  |
| mixing_error_k [K] | 1.93 | 1.86 | 0.95 |  |  |
| mixing_max_overshoot_k [K] | 0.867 | 0.837 | -0.444 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.124 | 0.123 | 0.0342 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 | at most 1 | pass |
| pump_run_hours [h] | 97.8 | 97 | 105 |  |  |
| radiator_heat_kwh [kWh] | 145 | 149 | 165 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 | - |  |  |
| tank_top_max_c [degC] | 84.6 | 84.6 | - |  |  |
| tank_top_min_c [degC] | 62.3 | 62.9 | - |  |  |
| valve_hours_closed [h] | 61.4 | 61.8 | 61.3 |  |  |
| valve_hours_open [h] | 38.6 | 35.5 | 3.58 |  |  |
| valve_mean_opening [-] | 0.502 | 0.51 | 0.492 |  |  |
| valve_movements_per_day [1/d] | 5.57 | 5.86 | 3.86 | at most 80 | pass |
| zone_max_c [degC] | 21.4 | 21.5 | 21.3 |  |  |
| zone_mean_occupied_c [degC] | 20.4 | 20.4 | 20.8 | 19.5 to 22.5 | pass |
| zone_min_c [degC] | 16.3 | 16.2 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 1.19 | 1.23 | 0.751 | at most 1.5 | pass |

{!validation/figures/systems_valve_pi.html!}

### `valve_onoff`

Radiator valve under an on-off controller. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.877 | 0.876 | 0.849 |  |  |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 31.6 | 32.6 | 60.2 |  |  |
| boiler_fuel_kwh [kWh] | 211 | 217 | 192 |  |  |
| boiler_gas_m3 [m3] | 18.1 | 18.6 | - |  |  |
| boiler_heat_kwh [kWh] | 185 | 190 | 163 |  |  |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - |  |  |
| boiler_shortest_firing_min [min] | 59 | 60.5 | 5 |  |  |
| boiler_starts_per_day [1/d] | 3.29 | 3.57 | 41 |  |  |
| boiler_water_firing_c [degC] | 71.1 | 71.4 | 68.6 |  |  |
| boiler_water_max_c [degC] | 87.7 | 87.6 | 69.8 |  |  |
| hours_below_setpoint [h] | 7.67 | 7.67 | 3.83 |  |  |
| mixing_error_k [K] | 4.74 | 5.17 | 1.42 |  |  |
| mixing_max_overshoot_k [K] | 0.915 | 0.786 | -0.432 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.0983 | 0.1 | 0.0326 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 | at most 1 | pass |
| pump_run_hours [h] | 75.3 | 77 | 57.9 |  |  |
| radiator_heat_kwh [kWh] | 155 | 160 | 166 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 | - |  |  |
| tank_top_max_c [degC] | 84.5 | 84.6 | - |  |  |
| tank_top_min_c [degC] | 61.5 | 62.5 | - |  |  |
| valve_hours_closed [h] | 92.7 | 91 | 108 |  |  |
| valve_hours_open [h] | 75.3 | 77 | 50.6 | at least 10 | pass |
| valve_mean_opening [-] | 0.448 | 0.458 | 0.328 |  |  |
| valve_movements_per_day [1/d] | 43.3 | 52.7 | 85.3 |  |  |
| zone_max_c [degC] | 21.5 | 21.2 | 21.8 |  |  |
| zone_mean_occupied_c [degC] | 20.8 | 20.9 | 20.9 | 19.5 to 22.5 | pass |
| zone_min_c [degC] | 16.5 | 16.5 | 16.6 |  |  |
| zone_tracking_rmse_k [K] | 0.546 | 0.531 | 0.622 | at most 1.5 | pass |

{!validation/figures/systems_valve_onoff.html!}

### `valve_nonlinear`

The proportional case with the valve's flow law not linearised (trano linearises it by default). Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| boiler_efficiency [-] | 0.877 | 0.876 | 0.846 |  |  |
| boiler_eta_firing [-] | 0.9 | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 31.7 | 32.5 | 96.5 |  |  |
| boiler_fuel_kwh [kWh] | 211 | 216 | 190 |  |  |
| boiler_gas_m3 [m3] | 18.1 | 18.5 | - |  |  |
| boiler_heat_kwh [kWh] | 185 | 190 | 161 |  |  |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | - |  |  |
| boiler_shortest_firing_min [min] | 74.6 | 58.4 | 10 |  |  |
| boiler_starts_per_day [1/d] | 3.29 | 3.57 | 4.57 |  |  |
| boiler_water_firing_c [degC] | 71.4 | 71.6 | 69.1 |  |  |
| boiler_water_max_c [degC] | 87.7 | 87.6 | 70 |  |  |
| hours_below_setpoint [h] | 5.08 | 5.42 | 4.08 |  |  |
| mixing_error_k [K] | 2.26 | 1.41 | 1.39 |  |  |
| mixing_max_overshoot_k [K] | 0.741 | 0.736 | -0.369 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| pump_electricity_kwh [kWh] | 0.123 | 0.133 | 0.0327 |  |  |
| pump_hours_without_demand [h] | 0 | 0 | 0 |  |  |
| pump_run_hours [h] | 95.5 | 105 | 98.3 |  |  |
| radiator_heat_kwh [kWh] | 156 | 159 | 161 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 | - |  |  |
| tank_top_max_c [degC] | 84.5 | 84.5 | - |  |  |
| tank_top_min_c [degC] | 61.3 | 63.2 | - |  |  |
| valve_hours_closed [h] | 69.9 | 63.3 | 64.6 |  |  |
| valve_hours_open [h] | 11.6 | 10.2 | 6.83 |  |  |
| valve_mean_opening [-] | 0.38 | 0.384 | 0.473 |  |  |
| valve_movements_per_day [1/d] | 23.6 | 18.4 | 23.9 |  |  |
| zone_max_c [degC] | 21.5 | 21.2 | 21.6 |  |  |
| zone_mean_occupied_c [degC] | 20.8 | 20.8 | 20.7 | 19.5 to 22.5 | pass |
| zone_min_c [degC] | 16.5 | 16.5 | 16.4 |  |  |
| zone_tracking_rmse_k [K] | 0.575 | 0.618 | 0.648 | at most 1.5 | pass |

{!validation/figures/systems_valve_nonlinear.html!}

## Ventilation

### `no_ventilation_winter`

The zone heated by an ideal heater, infiltration only (n50 = 3). Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| heating_kwh [kWh] | 138 | 140 | 144 | 80 to 220 | pass |
| heating_peak_w [W] | 4014 | 4066 | 4045 |  |  |
| hours_below_setpoint [h] | 1.08 | 1.17 | 1.08 | at most 10 | pass |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| zone_max_c [degC] | 21.4 | 21.5 | 21.4 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.9 | 20.9 |  |  |
| zone_min_c [degC] | 16 | 16 | 16 |  |  |

{!validation/figures/systems_no_ventilation_winter.html!}

### `hrv_winter`

Balanced ventilation with 80 % heat recovery (system_d), 150 m3/h, the coldest week. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| heating_kwh [kWh] | 164 | 165 | 171 | 100 to 260 | pass |
| heating_peak_w [W] | 4199 | 4240 | 4233 |  |  |
| hours_below_setpoint [h] | 1.08 | 1.17 | 1.08 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| vent_bypass_hours [h] | 0 | 0 | - |  |  |
| vent_effectiveness_measured [-] | 0.837 | 0.838 | 0.829 | 0.78 to 0.9 | pass |
| vent_fan_electricity_kwh [kWh] | 7.2 | 7.2 | 5.6 | 2 to 12 | pass |
| vent_fan_power_w [W] | 42.8 | 42.8 | 33.3 |  |  |
| vent_flow_kg_s [kg/s] | 0.05 | 0.05 | 0.05 |  |  |
| vent_heat_loss_kwh [kWh] | 28.4 | 28.2 | 29.8 | 15 to 60 | pass |
| vent_heater_electricity_kwh [kWh] | 0 | 0 | - |  |  |
| vent_hours_supply_above_zone [h] | 0 | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 143 | 142 | 144 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 1028 | 1028 | 800 | 400 to 2000 | pass |
| vent_supply_mean_c [degC] | 16.2 | 16.1 | 16 | 14 to 20 | pass |
| vent_supply_min_c [degC] | 12.8 | 12.8 | 12.6 |  |  |
| zone_max_c [degC] | 21.4 | 21.5 | 21.4 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.9 | 20.9 |  |  |
| zone_min_c [degC] | 16 | 16 | 16 |  |  |

{!validation/figures/systems_hrv_winter.html!}

### `balanced_no_recovery_winter`

The same balanced ventilation without heat recovery (effectiveness 0). Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| heating_kwh [kWh] | 298 | 300 | 305 |  |  |
| heating_peak_w [W] | 4961 | 5009 | 4985 |  |  |
| hours_below_setpoint [h] | 1.17 | 1.17 | 1.17 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| vent_bypass_hours [h] | 0 | 0 | - |  |  |
| vent_effectiveness_measured [-] | 0.0216 | 0.0221 | 0.0165 | at most 0.1 | pass |
| vent_fan_electricity_kwh [kWh] | 7.2 | 7.2 | 5.6 |  |  |
| vent_fan_power_w [W] | 42.8 | 42.8 | 33.3 |  |  |
| vent_flow_kg_s [kg/s] | 0.05 | 0.05 | 0.05 |  |  |
| vent_heat_loss_kwh [kWh] | 168 | 168 | 169 | 120 to 240 | pass |
| vent_heater_electricity_kwh [kWh] | 0 | 0 | - |  |  |
| vent_hours_supply_above_zone [h] | 0 | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 0 | 0 | 2.8 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 1028 | 1028 | 800 |  |  |
| vent_supply_mean_c [degC] | -0.666 | -0.656 | -0.764 | -2 to 2 | pass |
| vent_supply_min_c [degC] | -4.23 | -4.22 | -4.33 |  |  |
| zone_max_c [degC] | 21.4 | 21.5 | 21.4 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.9 | 20.9 |  |  |
| zone_min_c [degC] | 15.8 | 15.8 | 15.8 |  |  |

{!validation/figures/systems_balanced_no_recovery_winter.html!}

### `no_ventilation_summer`

The zone in July without ventilation (free floating above the set point). Week from day 196 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| heating_kwh [kWh] | 0.191 | 1.14 | 0.213 | at most 5 | pass |
| heating_peak_w [W] | 762 | 1357 | 795 |  |  |
| hours_below_setpoint [h] | 0.0833 | 0.25 | 0.0833 |  |  |
| outdoor_mean_c [degC] | 20 | 20 | 20 |  |  |
| zone_max_c [degC] | 25.2 | 25.6 | 25.1 |  |  |
| zone_mean_occupied_c [degC] | 22.9 | 22.8 | 22.8 |  |  |
| zone_min_c [degC] | 20.3 | 19.4 | 20.3 |  |  |

{!validation/figures/systems_no_ventilation_summer.html!}

### `hrv_summer`

Balanced ventilation with heat recovery in July: without a bypass it recovers heat it should not. Week from day 196 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| heating_kwh [kWh] | 0.301 | 1.43 | 0.374 |  |  |
| heating_peak_w [W] | 904 | 1454 | 955 |  |  |
| hours_below_setpoint [h] | 0.167 | 0.25 | 0.25 |  |  |
| outdoor_mean_c [degC] | 20 | 20 | 20 |  |  |
| vent_bypass_hours [h] | 0 | 0 | - |  |  |
| vent_effectiveness_measured [-] | 0.968 | 0.976 | 0.931 |  |  |
| vent_fan_electricity_kwh [kWh] | 7.2 | 7.2 | 5.6 |  |  |
| vent_fan_power_w [W] | 42.8 | 42.8 | 33.3 |  |  |
| vent_flow_kg_s [kg/s] | 0.05 | 0.05 | 0.05 |  |  |
| vent_heat_loss_kwh [kWh] | -2.34 | -2.66 | -1.14 |  |  |
| vent_heater_electricity_kwh [kWh] | 0 | 0 | - |  |  |
| vent_hours_supply_above_zone [h] | 95 | 96.6 | 88.4 | at least 10 | pass |
| vent_recovered_kwh [kWh] | 19.3 | 18 | 20.7 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 1028 | 1028 | 800 |  |  |
| vent_supply_mean_c [degC] | 22.7 | 22.6 | 22.5 |  |  |
| vent_supply_min_c [degC] | 18.6 | 18 | 18.4 |  |  |
| zone_max_c [degC] | 25.5 | 25.8 | 25.3 |  |  |
| zone_mean_occupied_c [degC] | 23 | 22.9 | 22.9 |  |  |
| zone_min_c [degC] | 20.1 | 19.3 | 20 |  |  |

{!validation/figures/systems_hrv_summer.html!}

### `hrv_bypass_summer`

The same unit with its summer bypass (open above 15 degC outdoor when the extract air is warmer). Week from day 196 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| heating_kwh [kWh] | 1.6 | 3.58 |  |  |
| heating_peak_w [W] | 1296 | 1589 |  |  |
| hours_below_setpoint [h] | 0.333 | 0.333 |  |  |
| outdoor_mean_c [degC] | 20 | 20 |  |  |
| vent_bypass_hours [h] | 79 | 75.1 | at least 20 | pass |
| vent_effectiveness_measured [-] | 0.303 | 0.31 |  |  |
| vent_fan_electricity_kwh [kWh] | 6.04 | 6.08 |  |  |
| vent_fan_power_w [W] | 36 | 36.2 |  |  |
| vent_flow_kg_s [kg/s] | 0.0509 | 0.0511 |  |  |
| vent_heat_loss_kwh [kWh] | 9.49 | 8.49 |  |  |
| vent_heater_electricity_kwh [kWh] | 0 | 0 |  |  |
| vent_hours_supply_above_zone [h] | 64.3 | 68.2 |  |  |
| vent_recovered_kwh [kWh] | 5.02 | 4.87 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 847 | 851 |  |  |
| vent_supply_mean_c [degC] | 20.9 | 20.9 |  |  |
| vent_supply_min_c [degC] | 14.1 | 14.1 |  |  |
| zone_max_c [degC] | 25.2 | 25.6 |  |  |
| zone_mean_occupied_c [degC] | 22.6 | 22.6 |  |  |
| zone_min_c [degC] | 19.7 | 19.3 |  |  |

{!validation/figures/systems_hrv_bypass_summer.html!}

### `hrv_night_setback_winter`

The unit at half its flow from 22 to 7 h (fan schedule). Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| heating_kwh [kWh] | 160 | 162 | 167 |  |  |
| heating_peak_w [W] | 4169 | 4215 | 4202 |  |  |
| hours_below_setpoint [h] | 1.08 | 1.17 | 1.08 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| vent_bypass_hours [h] | 0 | 0 | - |  |  |
| vent_effectiveness_measured [-] | 0.83 | 0.831 | 0.829 |  |  |
| vent_fan_electricity_kwh [kWh] | 5.41 | 5.41 | 4.72 | at most 10 | pass |
| vent_fan_power_w [W] | 32.2 | 32.2 | 28.1 |  |  |
| vent_flow_kg_s [kg/s] | 0.0421 | 0.0421 | 0.0421 | 0.038 to 0.045 | pass |
| vent_heat_loss_kwh [kWh] | 24.7 | 24.5 | 25.3 |  |  |
| vent_heater_electricity_kwh [kWh] | 0 | 0 | - |  |  |
| vent_hours_supply_above_zone [h] | 0 | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 120 | 120 | 122 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 917 | 917 | 801 |  |  |
| vent_supply_mean_c [degC] | 16.1 | 16 | 16 |  |  |
| vent_supply_min_c [degC] | 12.5 | 12.4 | 12.7 |  |  |
| zone_max_c [degC] | 21.4 | 21.5 | 21.4 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.9 | 20.9 |  |  |
| zone_min_c [degC] | 16 | 16 | 16 |  |  |

{!validation/figures/systems_hrv_night_setback_winter.html!}

### `hrv_postheater_winter`

The unit with an electric post-heater holding the supply air at 18 degC. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| heating_kwh [kWh] | 153 | 155 |  |  |
| heating_peak_w [W] | 3819 | 3886 |  |  |
| hours_below_setpoint [h] | 1.08 | 1.17 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| vent_bypass_hours [h] | 0 | 0 |  |  |
| vent_effectiveness_measured [-] | 0.923 | 0.927 |  |  |
| vent_fan_electricity_kwh [kWh] | 7.2 | 7.2 |  |  |
| vent_fan_power_w [W] | 42.8 | 42.8 |  |  |
| vent_flow_kg_s [kg/s] | 0.05 | 0.05 |  |  |
| vent_heat_loss_kwh [kWh] | 14.4 | 13.6 |  |  |
| vent_heater_electricity_kwh [kWh] | 14.3 | 15 | 2 to 40 | pass |
| vent_hours_supply_above_zone [h] | 43.4 | 47.9 |  |  |
| vent_recovered_kwh [kWh] | 144 | 143 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 1028 | 1028 |  |  |
| vent_supply_mean_c [degC] | 18 | 18 |  |  |
| vent_supply_min_c [degC] | 17.9 | 17.9 | at least 17 | pass |
| zone_max_c [degC] | 21.4 | 21.4 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.9 |  |  |
| zone_min_c [degC] | 16.3 | 16.2 |  |  |

{!validation/figures/systems_hrv_postheater_winter.html!}

### `extract_only_winter`

Mechanical extract ventilation (system_c), 150 m3/h: outdoor air through the trickle vents, one fan. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| heating_kwh [kWh] | 302 | 304 |  |  |
| heating_peak_w [W] | 4983 | 5031 |  |  |
| hours_below_setpoint [h] | 1.17 | 1.17 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| vent_effectiveness_measured [-] | 0 | 0 | at most 0.05 | pass |
| vent_fan_electricity_kwh [kWh] | 1.22 | 1.51 |  |  |
| vent_fan_power_w [W] | 7.24 | 8.97 |  |  |
| vent_flow_kg_s [kg/s] | 0.05 | 0.05 |  |  |
| vent_heat_loss_kwh [kWh] | 172 | 171 | 120 to 240 | pass |
| vent_hours_supply_above_zone [h] | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 0 | 0 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 174 | 215 | 150 to 1000 | pass |
| vent_supply_mean_c [degC] | -1.09 | -1.09 | -2 to 2 | pass |
| vent_supply_min_c [degC] | -4.66 | -4.66 |  |  |
| zone_max_c [degC] | 21.4 | 21.5 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.9 |  |  |
| zone_min_c [degC] | 15.8 | 15.8 |  |  |

{!validation/figures/systems_extract_only_winter.html!}

### `natural_winter`

Natural ventilation (system A): 150 m3/h of outdoor air through the vents and windows all day, no fan. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| heating_kwh [kWh] | 302 | 304 | 308 | 180 to 400 | pass |
| heating_peak_w [W] | 4983 | 5034 | 5002 |  |  |
| hours_below_setpoint [h] | 1.17 | 1.17 | 1.17 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 | -1.09 |  |  |
| zone_max_c [degC] | 21.4 | 21.5 | 21.4 |  |  |
| zone_mean_occupied_c [degC] | 20.9 | 20.9 | 20.9 |  |  |
| zone_min_c [degC] | 15.8 | 15.8 | 15.8 |  |  |

{!validation/figures/systems_natural_winter.html!}

### `ahu_g36_winter`

The Guideline 36 multizone air handling unit of trano (default variant) with a VAV box: it has no coils. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| heating_kwh [kWh] | 138 | 140 | 80 to 220 | pass |
| heating_peak_w [W] | 4163 | 4206 |  |  |
| hours_below_setpoint [h] | 0.933 | 0.9 |  |  |
| outdoor_mean_c [degC] | -1.09 | -1.09 |  |  |
| vent_effectiveness_measured [-] | 1.02 | 1.03 |  |  |
| vent_fan_electricity_kwh [kWh] | -1.3e-07 | 1.33e-21 |  |  |
| vent_fan_power_w [W] | -7.73e-07 | 7.91e-21 |  |  |
| vent_flow_kg_s [kg/s] | 4.44e-09 | 8.57e-16 | at most 0.001 | pass |
| vent_heat_loss_kwh [kWh] | -2.32e-07 | -2.53e-14 |  |  |
| vent_hours_supply_above_zone [h] | 60.4 | 62.7 |  |  |
| vent_recovered_kwh [kWh] | 1.59e-05 | 2.81e-12 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | -209 | 1.11e-05 |  |  |
| vent_supply_mean_c [degC] | 20 | 20 |  |  |
| vent_supply_min_c [degC] | 20 | 20 |  |  |
| zone_max_c [degC] | 21.4 | 21.5 |  |  |
| zone_mean_occupied_c [degC] | 21 | 21 |  |  |
| zone_min_c [degC] | 16 | 16 |  |  |

{!validation/figures/systems_ahu_g36_winter.html!}

## House

### `house_air_water_heat_pump`

Six-zone house: air-water heat pump (4 kW at -8/45 degC, 150 l buffer, heating curve), one mixing circuit on radiators rated 45/35 under proportional valves, balanced ventilation with heat recovery at half flow at night. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| hours_below_setpoint [h] | 0 | 0 | 170 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 39.9 | 39.9 | 39.7 |  |  |
| hp_condenser_out_running_c [degC] | 43.7 | 43.7 | 43 |  |  |
| hp_cop_ratio [-] | 0.892 | 0.895 | 0.999 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.71 | 2.73 | 3.03 |  |  |
| hp_cop_table [-] | 3.04 | 3.04 | 3.03 |  |  |
| hp_electricity_kwh [kWh] | 97.6 | 89.9 | 75.8 |  |  |
| hp_heat_kwh [kWh] | 294 | 271 | 224 | 150 to 700 | pass |
| hp_pump_share [-] | 0.00282 | 0.0028 | - |  |  |
| hp_run_hours [h] | 96 | 86.6 | 76.5 |  |  |
| hp_scop [-] | 3.01 | 3.02 | 2.95 | 2.4 to 3.8 | pass |
| hp_shortest_run_min [min] | 5 | 5 | 15 | at least 5 | pass |
| hp_shortest_stop_min [min] | 26.6 | 25.1 | 15 |  |  |
| hp_source_running_c [degC] | 0.589 | 0.618 | 0.129 |  |  |
| hp_speed_max [-] | 0.922 | 0.915 | 1 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 | 0.0103 |  |  |
| hp_starts_per_day [1/d] | 13.4 | 12.9 | 12.9 |  |  |
| hp_supply_mean_running_c [degC] | 39.1 | 39.1 | - |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 39.7 | 39.7 | 39.7 |  |  |
| mixing_error_k [K] | 5.4 | 5.41 | 5.43 |  |  |
| mixing_max_overshoot_k [K] | 0.352 | 0.356 | -0.762 |  |  |
| outdoor_mean_c [degC] | 0.488 | 0.488 | 0.488 |  |  |
| pump_electricity_kwh [kWh] | 0.624 | 0.575 | 0.144 |  |  |
| pump_hours_without_demand [h] | 115 | 121 | 67 |  |  |
| pump_run_hours [h] | 299 | 279 | 248 |  |  |
| radiator_heat_kwh [kWh] | 40.8 | 38.5 | 32 |  |  |
| valve_hours_closed [h] | 152 | 178 | 100 |  |  |
| valve_hours_open [h] | 9.12 | 7.34 | 172 |  |  |
| valve_mean_opening [-] | 0.34 | 0.311 | 0.6 |  |  |
| valve_movements_per_day [1/d] | 22.5 | 19.8 | 23.6 |  |  |
| vent_bypass_hours [h] | 0 | 0 | - |  |  |
| vent_effectiveness_measured [-] | 0.771 | 0.769 | 0.837 | 0.74 to 0.92 | pass |
| vent_fan_electricity_kwh [kWh] | 21.7 | 21.7 | 18.9 |  |  |
| vent_fan_power_w [W] | 64.5 | 64.5 | 56.2 |  |  |
| vent_flow_kg_s [kg/s] | 0.0838 | 0.0842 | 0.0842 | 0.075 to 0.09 | pass |
| vent_heat_loss_kwh [kWh] | 125 | 128 | 109 |  |  |
| vent_heater_electricity_kwh [kWh] | 0 | 0 | - |  |  |
| vent_hours_supply_above_zone [h] | 0 | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 429 | 431 | 403 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 924 | 920 | 802 |  |  |
| vent_supply_mean_c [degC] | 15.9 | 16 | 14.6 |  |  |
| vent_supply_min_c [degC] | 12.4 | 12.6 | 12 |  |  |
| zone_max_c [degC] | 22 | 22 | 21.7 |  |  |
| zone_mean_occupied_c [degC] | 21.9 | 21.9 | 19.3 | 18.5 to 23 | pass |
| zone_min_c [degC] | 16 | 16.4 | 15.8 |  |  |
| zone_tracking_rmse_k [K] | 0.858 | 0.865 | 2.01 |  |  |

{!validation/figures/systems_house_air_water_heat_pump.html!}

### `house_ground_source_heat_pump`

Six-zone house: ground-source heat pump (Carnot, 10 degC ground loop), the same circuit and radiators, balanced ventilation with heat recovery. Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| hours_below_setpoint [h] | 0 | 0 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0.0833 |  |  |
| hp_buffer_mean_c [degC] | 39.9 | 39.9 |  |  |
| hp_condenser_out_running_c [degC] | 44.7 | 44.7 |  |  |
| hp_cop_ratio [-] | 0.806 | 0.807 |  |  |
| hp_cop_running [-] | 3.04 | 3.05 |  |  |
| hp_cop_table [-] | 3.78 | 3.78 |  |  |
| hp_electricity_kwh [kWh] | 97.8 | 90.4 |  |  |
| hp_heat_kwh [kWh] | 307 | 284 | 150 to 700 | pass |
| hp_pump_share [-] | 0.00292 | 0.00288 |  |  |
| hp_run_hours [h] | 85.5 | 78.9 |  |  |
| hp_scop [-] | 3.14 | 3.14 | 2.8 to 4.6 | pass |
| hp_shortest_run_min [min] | 5 | 5 |  |  |
| hp_shortest_stop_min [min] | 24.7 | 25.3 |  |  |
| hp_source_running_c [degC] | 7.19 | 7.19 | 6.5 to 8.5 | pass |
| hp_speed_max [-] | 1 | 1 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 |  |  |
| hp_starts_per_day [1/d] | 15.5 | 14.9 |  |  |
| hp_supply_mean_running_c [degC] | 39.1 | 39.1 |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 39.7 | 39.7 |  |  |
| mixing_error_k [K] | 5.34 | 5.35 |  |  |
| mixing_max_overshoot_k [K] | 0.34 | 0.351 |  |  |
| outdoor_mean_c [degC] | 0.488 | 0.488 |  |  |
| pump_electricity_kwh [kWh] | 0.634 | 0.587 |  |  |
| pump_hours_without_demand [h] | 110 | 123 |  |  |
| pump_run_hours [h] | 299 | 283 |  |  |
| radiator_heat_kwh [kWh] | 42 | 39.7 |  |  |
| valve_hours_closed [h] | 147 | 176 |  |  |
| valve_hours_open [h] | 10.6 | 8.17 |  |  |
| valve_mean_opening [-] | 0.35 | 0.32 |  |  |
| valve_movements_per_day [1/d] | 22.8 | 20.9 |  |  |
| vent_bypass_hours [h] | 0 | 0 |  |  |
| vent_effectiveness_measured [-] | 0.778 | 0.776 | 0.74 to 0.92 | pass |
| vent_fan_electricity_kwh [kWh] | 28.8 | 28.8 |  |  |
| vent_fan_power_w [W] | 85.6 | 85.6 |  |  |
| vent_flow_kg_s [kg/s] | 0.1 | 0.1 |  |  |
| vent_heat_loss_kwh [kWh] | 146 | 150 |  |  |
| vent_heater_electricity_kwh [kWh] | 0 | 0 |  |  |
| vent_hours_supply_above_zone [h] | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 511 | 513 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 1027 | 1027 |  |  |
| vent_supply_mean_c [degC] | 16 | 16.1 |  |  |
| vent_supply_min_c [degC] | 12.7 | 12.9 |  |  |
| zone_max_c [degC] | 22 | 22 |  |  |
| zone_mean_occupied_c [degC] | 21.9 | 21.9 | 18.5 to 23 | pass |
| zone_min_c [degC] | 16 | 16.5 |  |  |
| zone_tracking_rmse_k [K] | 0.853 | 0.859 |  |  |

{!validation/figures/systems_house_ground_source_heat_pump.html!}

### `house_condensing_boiler`

Six-zone house: condensing boiler without storage at 45 degC on the 45/35 radiators, mechanical extract ventilation (system C). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| boiler_efficiency [-] | 0.948 | 0.948 | 0.88 to 0.99 | pass |
| boiler_eta_firing [-] | 0.957 | 0.957 | 0.93 to 0.99 | pass |
| boiler_firing_hours [h] | 332 | 329 |  |  |
| boiler_fuel_kwh [kWh] | 847 | 818 |  |  |
| boiler_gas_m3 [m3] | 72.6 | 70.1 |  |  |
| boiler_heat_kwh [kWh] | 803 | 775 | 250 to 1000 | pass |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | 11.6 to 11.75 | pass |
| boiler_shortest_firing_min [min] | 3.23 | 3.46 |  |  |
| boiler_starts_per_day [1/d] | 0.286 | 0.643 |  |  |
| boiler_water_firing_c [degC] | 44.7 | 44.7 |  |  |
| boiler_water_max_c [degC] | 45 | 45 |  |  |
| hours_below_setpoint [h] | 144 | 153 |  |  |
| mixing_error_k [K] | 0.326 | 0.32 |  |  |
| mixing_max_overshoot_k [K] | -0.0181 | -0.0179 |  |  |
| outdoor_mean_c [degC] | 0.488 | 0.488 |  |  |
| pump_electricity_kwh [kWh] | 0.825 | 0.82 |  |  |
| pump_hours_without_demand [h] | 9.24 | 10.3 |  |  |
| pump_run_hours [h] | 336 | 335 |  |  |
| radiator_heat_kwh [kWh] | 120 | 119 |  |  |
| valve_hours_closed [h] | 9.5 | 11.2 |  |  |
| valve_hours_open [h] | 122 | 122 |  |  |
| valve_mean_opening [-] | 0.781 | 0.772 |  |  |
| valve_movements_per_day [1/d] | 6 | 5.57 |  |  |
| vent_effectiveness_measured [-] | 0 | 0 | at most 0.05 | pass |
| vent_fan_electricity_kwh [kWh] | 5.11 | 6.01 |  |  |
| vent_fan_power_w [W] | 15.2 | 17.9 |  |  |
| vent_flow_kg_s [kg/s] | 0.0999 | 0.0999 |  |  |
| vent_heat_loss_kwh [kWh] | 613 | 611 |  |  |
| vent_hours_supply_above_zone [h] | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 0 | 0 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 183 | 215 | 100 to 1000 | pass |
| vent_supply_mean_c [degC] | 0.488 | 0.488 |  |  |
| vent_supply_min_c [degC] | -4.66 | -4.66 |  |  |
| zone_max_c [degC] | 21.8 | 21.8 |  |  |
| zone_mean_occupied_c [degC] | 20 | 19.9 | 18.5 to 23 | pass |
| zone_min_c [degC] | 15.8 | 15.9 |  |  |
| zone_tracking_rmse_k [K] | 1.39 | 1.46 |  |  |

{!validation/figures/systems_house_condensing_boiler.html!}

### `house_boiler_storage`

Six-zone house: gas boiler with a 200 l storage tank at 70 degC, radiators re-rated 70/50 under on-off valves (1 % leakage so that the loop keeps a flow path when they all close), natural ventilation through the windows (system A). Week from day 18 of the year.

| KPI | Buildings | IDEAS | Band | Status |
|---|---|---|---|---|
| boiler_efficiency [-] | 0.883 | 0.883 | 0.8 to 0.91 | pass |
| boiler_eta_firing [-] | 0.9 | 0.9 |  |  |
| boiler_firing_hours [h] | 109 | 105 |  |  |
| boiler_fuel_kwh [kWh] | 966 | 933 |  |  |
| boiler_gas_m3 [m3] | 82.8 | 80 |  |  |
| boiler_heat_kwh [kWh] | 853 | 824 | 250 to 1000 | pass |
| boiler_kwh_per_m3 [kWh/m3] | 11.7 | 11.7 | 11.6 to 11.75 | pass |
| boiler_shortest_firing_min [min] | 44.3 | 41.3 |  |  |
| boiler_starts_per_day [1/d] | 7.79 | 7.57 | 1 to 40 | pass |
| boiler_water_firing_c [degC] | 72.1 | 72.1 |  |  |
| boiler_water_max_c [degC] | 87.3 | 87.4 |  |  |
| hours_below_setpoint [h] | 167 | 159 |  |  |
| mixing_error_k [K] | 2.26 | 2.43 |  |  |
| mixing_max_overshoot_k [K] | 1.51 | 1.48 |  |  |
| outdoor_mean_c [degC] | 0.488 | 0.488 |  |  |
| pump_electricity_kwh [kWh] | 0.654 | 0.644 |  |  |
| pump_hours_without_demand [h] | 108 | 110 |  |  |
| pump_run_hours [h] | 299 | 294 |  |  |
| radiator_heat_kwh [kWh] | 122 | 119 |  |  |
| tank_bottom_max_c [degC] | 75.1 | 75.1 |  |  |
| tank_top_max_c [degC] | 84 | 84.4 |  |  |
| tank_top_min_c [degC] | 61.6 | 60.9 |  |  |
| valve_hours_closed [h] | 145 | 151 |  |  |
| valve_hours_open [h] | 191 | 185 |  |  |
| valve_mean_opening [-] | 0.567 | 0.55 |  |  |
| valve_movements_per_day [1/d] | 136 | 135 |  |  |
| zone_max_c [degC] | 22.2 | 22.2 |  |  |
| zone_mean_occupied_c [degC] | 19.7 | 20 | 18.5 to 23 | pass |
| zone_min_c [degC] | 15.8 | 15.9 |  |  |
| zone_tracking_rmse_k [K] | 1.84 | 1.58 |  |  |

{!validation/figures/systems_house_boiler_storage.html!}

### `house_heat_pump_pv`

Six-zone house: the air-water heat pump of house_air_water_heat_pump with 20 m2 of photovoltaics on the roof and balanced ventilation with heat recovery. Week from day 18 of the year.

| KPI | Buildings | IDEAS | FastSmoothBuildings | Band | Status |
|---|---|---|---|---|---|
| hours_below_setpoint [h] | 0 | 0 | 168 |  |  |
| hp_buffer_below_band_hours [h] | 0 | 0 | 0 |  |  |
| hp_buffer_mean_c [degC] | 39.9 | 39.9 | 39.7 |  |  |
| hp_condenser_out_running_c [degC] | 43.7 | 43.7 | 42.9 |  |  |
| hp_cop_ratio [-] | 0.888 | 0.887 | 0.996 | 0.85 to 1.1 | pass |
| hp_cop_running [-] | 2.7 | 2.7 | 3.03 |  |  |
| hp_cop_table [-] | 3.04 | 3.04 | 3.04 |  |  |
| hp_electricity_kwh [kWh] | 102 | 94.5 | 82 |  |  |
| hp_heat_kwh [kWh] | 307 | 285 | 248 | 150 to 700 | pass |
| hp_pump_share [-] | 0.0028 | 0.00279 | - |  |  |
| hp_run_hours [h] | 101 | 91.1 | 85.8 |  |  |
| hp_scop [-] | 3 | 3.01 | 3.03 | 2.4 to 3.8 | pass |
| hp_shortest_run_min [min] | 5 | 10.7 | 15 |  |  |
| hp_shortest_stop_min [min] | 24.3 | 21.9 | 15 |  |  |
| hp_source_running_c [degC] | 0.563 | 0.59 | 0.189 |  |  |
| hp_speed_max [-] | 0.938 | 0.916 | 0.977 |  |  |
| hp_speed_min_running [-] | 0.3 | 0.3 | 0.0182 |  |  |
| hp_starts_per_day [1/d] | 13.6 | 13.4 | 13.5 |  |  |
| hp_supply_mean_running_c [degC] | 39.1 | 39.1 | - |  |  |
| hp_supply_setpoint_max_c [degC] | 42.9 | 42.9 | 42.9 |  |  |
| hp_supply_setpoint_mean_c [degC] | 39.7 | 39.7 | 39.7 |  |  |
| mixing_error_k [K] | 5.37 | 5.4 | 5.43 |  |  |
| mixing_max_overshoot_k [K] | 0.339 | 0.355 | -0.762 |  |  |
| outdoor_mean_c [degC] | 0.488 | 0.488 | 0.488 |  |  |
| pump_electricity_kwh [kWh] | 0.633 | 0.588 | 0.159 |  |  |
| pump_hours_without_demand [h] | 110 | 123 | 67.5 |  |  |
| pump_run_hours [h] | 299 | 283 | 262 |  |  |
| radiator_heat_kwh [kWh] | 42.1 | 39.7 | 34.9 |  |  |
| valve_hours_closed [h] | 146 | 176 | 99 |  |  |
| valve_hours_open [h] | 9.29 | 7.75 | 168 |  |  |
| valve_mean_opening [-] | 0.348 | 0.319 | 0.609 |  |  |
| valve_movements_per_day [1/d] | 21.1 | 19.6 | 23.1 |  |  |
| vent_bypass_hours [h] | 0 | 0 | - |  |  |
| vent_effectiveness_measured [-] | 0.778 | 0.775 | 0.837 |  |  |
| vent_fan_electricity_kwh [kWh] | 28.8 | 28.8 | 22.4 |  |  |
| vent_fan_power_w [W] | 85.6 | 85.6 | 66.7 |  |  |
| vent_flow_kg_s [kg/s] | 0.1 | 0.1 | 0.1 |  |  |
| vent_heat_loss_kwh [kWh] | 146 | 150 | 127 |  |  |
| vent_heater_electricity_kwh [kWh] | 0 | 0 | - |  |  |
| vent_hours_supply_above_zone [h] | 0 | 0 | 0 |  |  |
| vent_recovered_kwh [kWh] | 511 | 514 | 478 |  |  |
| vent_sfp_w_per_m3s [W/(m3/s)] | 1028 | 1028 | 800 |  |  |
| vent_supply_mean_c [degC] | 16 | 16.1 | 14.6 |  |  |
| vent_supply_min_c [degC] | 12.7 | 12.9 | 12.1 |  |  |
| zone_max_c [degC] | 22 | 22 | 21.6 |  |  |
| zone_mean_occupied_c [degC] | 21.9 | 21.9 | 19.3 | 18.5 to 23 | pass |
| zone_min_c [degC] | 16 | 16.5 | 15.7 |  |  |
| zone_tracking_rmse_k [K] | 0.853 | 0.86 | 1.99 |  |  |

{!validation/figures/systems_house_heat_pump_pv.html!}
