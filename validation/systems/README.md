# Systems validation

Do the heat pumps, gas boilers, valves, pumps and ventilation units trano generates behave as the real ones
do? Each case puts one system on the same 100 m2 study zone (`models.py`) for one week of the Uccle weather
(the coldest week, a mild March week, a July week), simulated with Buildings, IDEAS and FastSmoothBuildings.
`kpi.py` reads the KPIs of the system from the result file (COP against the EN 14511 table, electricity and gas,
cycling, supply temperatures, valve openings, fan power, recovered heat...), `expected.yaml` holds the band of
every KPI with the physical reasoning behind it, and `pytest -m systems` fails when a KPI leaves its band.
`PLAN.md` is the plan of the campaign and its findings.

```bash
python -m validation.systems list [--family heat_pump]        # the cases
python -m validation.systems generate                          # rewrite cases/*.yaml after a change of cases.py
python -m validation.systems run hp_winter --library Buildings # simulate and extract the KPIs
python -m validation.systems run --family boiler --workers 2   # a family with the three libraries
python -m validation.systems report [--docs]                   # Markdown (and docs/validation/systems.md)
python -m validation.systems commit --library Buildings        # keep the results in results/<library>
pytest -m systems                                              # the gate
```

The runs use the runtime of `trano_simulate`: Docker by default, or the `omc` of the machine with
`TRANO_SIMULATION_RUNTIME=local` (the libraries installed as OpenModelica's package manager lays them out). Results
are cached in `.cache/systems/<library>/<case>` by the hash of the generated model: a case whose model did not
change is read back; a case whose KPI code changed is re-read from its result file without simulating again.
Run the libraries in parallel (one process each) rather than several cases of one library at a time with the
local runtime: concurrent `omc` translations in the same OpenModelica home collide. A house model takes `omc` to
about 6 GB while it translates, so on a machine with less than 16 GB run the house family one library at a time.

Adding a case: a builder in `cases.py` (the YAML blocks come from `models.py`), `generate`, the bands in
`expected.yaml`, and a KPI function in `kpi.py` when the family is new. A band names the libraries it holds
for when one library cannot provide the KPI.
