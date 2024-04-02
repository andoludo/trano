docker run -v ./model:/model -it openmodelica/openmodelica:v1.22.0-ompython bash


omc --simulation model.mo ./libraries/ModelicaStandardLibrary/Modelica/package.mo './libraries/Buildings 10.0.0/package.mo'


## pre-commit

  sh "pip install pre-commit"
  sh "git config --global --add safe.directory '*'"
  sh "git config --unset-all core.hooksPath"
  sh "pre-commit install"
  sh "pre-commit run --all-files --config .pre-commit-config.yaml"
