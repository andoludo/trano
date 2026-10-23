# FastModel against Buildings: BESTEST cases and a renovated house

A study of how close the `FastModel` library stays to Buildings on the zone temperatures, the heating power and the
heating demand, and of whether a parameter change moves both libraries alike. The cases (`cases.py`):

| Case | What it is | Period |
|---|---|---|
| 950FF | BESTEST heavy zone, free floating, night ventilation (1700 m3/h, 18:00 to 07:00) | a year |
| 940 | BESTEST heavy zone, ideal heating with a night set-back and cooling | a year |
| 960 | BESTEST light zone behind an unconditioned heavy sun-space, heated and cooled | a year |
| residential_after | `examples/residential_after_renovation.yaml`: six zones, heat pump, buffer, radiators, heat recovery, PV | 28 days of January |

Each case has *perturbations*: one field of the YAML changed (the conductivity of an insulation, the infiltration, the
thermal mass, the glazing, the solar absorptance, the gains, the rating of the heat pump, a set point, ...) and
simulated with both libraries, so that the response of each library to the same change can be compared. Some
parameters are *swept* (`sweep` in `cases.py`: the same field scaled by several factors), to see whether the two
libraries keep agreeing over a range of values. The report opens with every response at a glance: the ratio of the
change of FastModel to the change of Buildings on each main KPI, and a verdict (alike, close, differs).

```bash
python -m validation.fast_model list
python -m validation.fast_model run 950FF                      # every variant, both libraries (Docker)
python -m validation.fast_model run residential_after --variants base eps_x1.5 --libraries FastModel
python -m validation.fast_model report --docs                  # docs/validation/fast_model_study.md and its figures
```

Both libraries are simulated by OpenModelica in the trano container (FastModel is a Modelica library, so the CasADi
export is not involved). The hourly series (`<library>.csv.gz`) and the KPIs (`<library>.json`) of every run are
committed under `results/<case>/<variant>/`; the report and the documentation page are rendered from them without
Docker. The workflow `.github/workflows/fast_model_study.yaml` runs the matrix and commits the results and the
rendered page to the branch it runs on: on demand from the default branch, or on a push to any branch whose commit
message carries a tag, `[study]` for every run or `[study 940 residential_after | base]` for the cases before the bar
and the variants after it. A change of the FastModel library (`trano/library/FastModel.mo`, its templates,
`library.json`) is expected to carry `[study]`, so that the study page follows the library.
