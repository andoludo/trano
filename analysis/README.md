# Analyses

Analyses of trano that go beyond the validation suites: what was asked, how it was answered, the scripts that
produce the numbers and the numbers themselves, committed so that they can be read without running anything.

| Folder | What it analyses |
|---|---|
| [`fast_model/`](fast_model/README.md) | the `FastModel` library (the smooth library that rumoca translates to CasADi for MPC): BESTEST, the comparison with Buildings on the same YAML, the MPC export and optimal control of the residential houses, shading, the boiler water loop, the open gaps |

Each folder holds numbered reports (Markdown), a `scripts/` package (`python -m analysis.<folder>.scripts.<name>`,
run from the root of the repository) and a `results/` folder with what the scripts wrote. The scripts read the
validation results committed under `validation/` where they can, and simulate (OpenModelica in Docker, as the
validation does) or export to CasADi (trano-optimization with rumoca) only where they must; each says which in its
docstring.
