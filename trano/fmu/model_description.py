"""The variables of an FMU, read from its ``modelDescription.xml``.

Only what the calibration inventory needs is kept: the name, value reference, type, causality,
variability, initial attribute, start value, unit and description of each scalar variable. The
reader is independent of the FMU runtime (FMPy is only needed to simulate), so it runs on the
checked-in fixture of the unit tests.
"""

import zipfile
from pathlib import Path
from xml.etree import ElementTree

from pydantic import BaseModel, ConfigDict, Field

from trano.exceptions import FmuError

MODEL_DESCRIPTION = "modelDescription.xml"
SCALAR_TYPES = ("Real", "Integer", "Boolean", "String", "Enumeration")


class FmuVariable(BaseModel):
    """One ``ScalarVariable`` of the model description (FMI 2.0)."""

    model_config = ConfigDict(frozen=True)
    name: str
    value_reference: int
    type: str = Field(description="Real, Integer, Boolean, String or Enumeration")
    causality: str = Field(default="local", description="parameter, calculatedParameter, input, output, local")
    variability: str = Field(default="continuous", description="constant, fixed, tunable, discrete, continuous")
    initial: str | None = Field(default=None, description="exact, approx or calculated when the exporter writes it")
    start: str | None = Field(default=None, description="Start value as written in the XML")
    unit: str | None = None
    description: str | None = None

    @property
    def is_parameter(self) -> bool:
        return self.causality == "parameter"

    @property
    def settable(self) -> bool:
        """Whether an importer may set the variable before initialisation (FMI 2.0 table of allowed combinations).

        A ``parameter`` with ``fixed`` or ``tunable`` variability has ``initial = exact`` and takes a value from the
        importer; a ``calculatedParameter`` is computed by the FMU from the others and a ``constant`` is evaluated
        at translation. ``fixed`` is enough for calibration, which re-instantiates the FMU at every run.
        """
        return self.causality == "parameter" and self.variability in ("fixed", "tunable")

    def start_value(self) -> float | None:
        """The start value as a number when it is one (Real, Integer, Boolean as 0/1)."""
        if self.start is None:
            return None
        if self.type == "Boolean":
            return 1.0 if self.start == "true" else 0.0
        try:
            return float(self.start)
        except ValueError:
            return None


class ModelDescription(BaseModel):
    """The scalar variables of an FMU with the attributes of its model."""

    model_config = ConfigDict(frozen=True)
    fmi_version: str
    model_name: str
    guid: str
    generation_tool: str | None = None
    co_simulation: bool = False
    model_exchange: bool = False
    variables: tuple[FmuVariable, ...]

    def by_name(self) -> dict[str, FmuVariable]:
        return {variable.name: variable for variable in self.variables}

    def get(self, name: str) -> FmuVariable | None:
        return self.by_name().get(name)

    def parameters(self) -> list[FmuVariable]:
        """The variables an importer may set before initialisation."""
        return [variable for variable in self.variables if variable.settable]

    def matching(self, prefix: str) -> list[FmuVariable]:
        """The variables of a component: ``prefix`` followed by a dot or an array index."""
        return [
            variable
            for variable in self.variables
            if variable.name == prefix or variable.name.startswith((f"{prefix}.", f"{prefix}["))
        ]


def parse_model_description(xml: str) -> ModelDescription:
    """Parse the text of a ``modelDescription.xml``."""
    root = ElementTree.fromstring(xml)  # noqa: S314 - the FMU is built by trano itself
    if root.tag != "fmiModelDescription":
        raise FmuError(f"Not a model description: root element {root.tag}")
    variables = []
    model_variables = root.find("ModelVariables")
    for scalar in model_variables if model_variables is not None else []:
        if scalar.tag != "ScalarVariable":
            continue
        typed = next((child for child in scalar if child.tag in SCALAR_TYPES), None)
        if typed is None:
            continue
        variables.append(
            FmuVariable(
                name=scalar.attrib["name"],
                value_reference=int(scalar.attrib["valueReference"]),
                type=typed.tag,
                causality=scalar.attrib.get("causality", "local"),
                variability=scalar.attrib.get("variability", "continuous"),
                initial=scalar.attrib.get("initial"),
                start=typed.attrib.get("start"),
                unit=typed.attrib.get("unit"),
                description=scalar.attrib.get("description"),
            )
        )
    return ModelDescription(
        fmi_version=root.attrib.get("fmiVersion", ""),
        model_name=root.attrib.get("modelName", ""),
        guid=root.attrib.get("guid", ""),
        generation_tool=root.attrib.get("generationTool"),
        co_simulation=root.find("CoSimulation") is not None,
        model_exchange=root.find("ModelExchange") is not None,
        variables=tuple(variables),
    )


def read_model_description(fmu: Path) -> ModelDescription:
    """Read the model description of an FMU (a zip archive) or of an extracted ``modelDescription.xml``."""
    if fmu.is_dir():
        fmu = fmu / MODEL_DESCRIPTION
    if fmu.suffix == ".xml":
        return parse_model_description(fmu.read_text())
    with zipfile.ZipFile(fmu) as archive:
        return parse_model_description(archive.read(MODEL_DESCRIPTION).decode())


def fmu_resources(fmu: Path) -> list[str]:
    """The files packed under ``resources/`` in the FMU, relative to that directory."""
    with zipfile.ZipFile(fmu) as archive:
        return sorted(
            name.removeprefix("resources/")
            for name in archive.namelist()
            if name.startswith("resources/") and not name.endswith("/")
        )
