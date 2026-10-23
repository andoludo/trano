# 8. The house before renovation (gas boiler)

**Question.** Does FastModel also work with `examples/residential_before_renovation.yaml`: the same six-zone
semi-detached house with uninsulated cavity walls and attic, old double glazing, infiltration only (n50 = 12 1/h), a
non-condensing gas boiler at 75 degC, a pump, a three-way valve and six radiators with thermostatic valves?

**Answer.** Yes, without any change of the library. In closed loop it follows Buildings within 0.02 to 0.09 K per
room hour by hour, its heating is 1.7 % above Buildings and its gas 1.9 %; every change of the YAML that moves the
results moves both libraries alike, gas included. As an MPC model it exports in 45 s (160 states) and IPOPT solves its
optimal control in 23 s ([MPC](06_mpc_rumoca_ipopt.md)).

Data: the `residential_before` rows of [`results/study_kpis.csv`](results/study_kpis.csv),
[`results/study_fidelity.csv`](results/study_fidelity.csv), [`results/study_responses.csv`](results/study_responses.csv),
[`results/study_verdicts.csv`](results/study_verdicts.csv) and [`results/study_hour_of_day.csv`](results/study_hour_of_day.csv);
[`results/mpc_residential_before_renovation.json`](results/mpc_residential_before_renovation.json). The study case
`residential_before` is in `validation/fast_model/cases.py`; its runs are in `validation/fast_model/results/residential_before`.

## Closed loop against Buildings, 28 days of January in Uccle

| Room | Hourly RMSE | Bias | Largest error | Heat of the radiators, CV(RMSE) | Heat, deviation |
|---|---|---|---|---|---|
| living | 0.02 K | +0.01 K | 0.22 K | 2.8 % | +2.3 % |
| kitchen | 0.03 K | +0.01 K | 0.32 K | 1.4 % | +1.0 % |
| bedroom 1 | 0.04 K | +0.01 K | 0.47 K | 2.6 % | +1.8 % |
| bedroom 2 | 0.04 K | +0.01 K | 0.45 K | 1.9 % | +1.5 % |
| bedroom 3 | 0.04 K | +0.01 K | 0.46 K | 1.8 % | +1.3 % |
| bathroom | 0.08 K | +0.01 K | 0.51 K | 3.1 % | +1.4 % |

The largest errors are in the first hours of the month, while both models settle from their initial temperatures;
after the first day no room is more than 0.32 K from Buildings.

| | Buildings | FastModel | |
|---|---|---|---|
| Heat of the radiators | 2755 kWh | 2802 kWh | +1.7 % |
| Peak heating (hourly) | 8.70 kW | 8.78 kW | +0.9 % |
| Gas (energy meters) | 3391 kWh | 3454 kWh | +1.9 % |
| OpenModelica time | 268 s | 112 s | 2.4 times faster |

## Responses to the YAML

| Change | Heating | Peak | Gas | Verdict |
|---|---|---|---|---|
| cavity of the walls filled (U 1.47 → 0.52 W/(m2.K)) | −23.3 % / −23.1 % (1.01) | −16.9 % / −16.9 % (1.01) | −22.2 % / −22.1 % (1.01) | alike |
| airtightness n50 x0.5 | −11.1 % / −11.0 % (1.00) | −8.0 % / −7.8 % (0.98) | −10.7 % / −10.5 % (1.01) | alike |
| airtightness n50 x1.5 | +11.2 % / +11.0 % (1.00) | +4.2 % / +3.7 % (0.88) | +10.7 % / +10.5 % (1.00) | alike |
| supply set point 65 instead of 75 degC | | −11.8 % / −12.4 % (1.06) | −1.5 % / −1.6 % | alike |
| radiators x0.7 | | −12.0 % / −12.6 % (1.06) | −1.0 % / −1.0 % | alike |
| living room and kitchen +1 K | +4.8 % / +4.7 % (0.99) | | +4.6 % / +4.5 % (0.99) | alike |
| air of the glazing gaps x0.5 | −1.0 % / −1.1 % | −0.8 % / −0.8 % | −1.0 % / −1.1 % | neither moves beyond 2 % |
| boiler 10 kW instead of 16 kW | −0.1 % / −0.1 % | −1.0 % / −1.3 % | −1.8 % / −1.8 % | neither moves beyond 2 % |
| radiator exponent 1.33 instead of 1.24 | 0 % / 0 % | −0.2 % / −0.4 % | 0 % / 0 % | neither moves beyond 2 % |

(Buildings / FastModel, and the ratio of the changes where a library moves by more than 2 %.) A smaller boiler saves
1.8 % of gas in both libraries for the same heat: the heat loss of the boiler to its room scales with its rating in
`BoilerPolynomial` and in FastModel's port of it.

## The gas

The gas is 1.9 % above Buildings for 1.7 % more heat: per unit of heat the two libraries burn the same gas within
0.2 % (1.230 and 1.233), and both water loops close their energy balance within 0.15 % over the month. The study first
reported +3.3 %, then +2.6 %: it read the gas of both libraries, then of Buildings only, from samples of the fuel
power five minutes apart, which catch or miss the two-minute bursts of the boilers when the valves open in the morning. It
now reads the energy meters of both. The [boiler water loop](09_boiler_water_loop.md) has the balances and the
restart.

## MPC

160 states, 27 inputs; controls: the boiler, the three-way valve and the six radiator valves; the gas meter `EFue` is
the cost. IPOPT holds the six rooms at 20 degC through a 12-hour night at 0 degC with the boiler at 0.19 to 0.32 of its
power and 57 kWh of gas ([MPC](06_mpc_rumoca_ipopt.md)). The closed loop exports too (224 states), and every MPC
state is a closed-loop state of the same name.
