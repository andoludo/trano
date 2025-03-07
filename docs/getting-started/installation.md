# Installation
## Optional dependencies

### Graphviz

Trano employs Graphviz to arrange various Modelica components within the model. While installation is not mandatory, it is advisable for enhanced model visualization. You can install Graphviz by following the instructions on the [official website](https://graphviz.org/download/). For Windows installations, ensure the Graphviz `bin` folder is added to the system PATH.

To install Graphviz on Linux, use the following command:

```bash
sudo apt-get install graphviz
```


```bash

sudo apt update
sudo apt install graphviz
                
```
            

### Docker

Trano utilizes the official OpenModelica Docker image to execute simulations. It is recommended to install Docker on your system so that Trano can pull the required images and run simulations. You can install Docker by following the instructions on the [official website](https://docs.docker.com/engine/install/).

## Python package


!!! warning
    Trano requires python 3.9 or higher and docker to be installed on the system.
            

Trano is a Python package that you can install via pip.


```bash
pip install trano
```
            

Trano can also be utilized with Poetry.


```bash
poetry add trano
```
            

The Python package enables the generation of a Modelica model from the YAML configuration input. However, to simulate the model, Docker must be installed on the system.

On Windows systems, you may need to enable long path support. Refer to the [Microsoft documentation](https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=registry) for guidance.

## Check installation

To verify the installation, execute the following command in the terminal.


```bash
trano verify
```
            

This command checks your system's compatibility for model generation and simulation. Compatibility is optional if you only intend to generate the Modelica model.

