docker run -v ./model:/model -it openmodelica/openmodelica:v1.22.0-ompython bash


omc --simulation model.mo ./libraries/ModelicaStandardLibrary/Modelica/package.mo './libraries/Buildings 10.0.0/package.mo'