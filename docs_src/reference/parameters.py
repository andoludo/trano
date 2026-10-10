"""The parameter reference: one table per parameter class, one column per library."""

from pathlib import Path

from trano.elements.library.parameters import PARAMETERS, ParameterSpec, parameter_specs

COLUMNS = {
    "buildings": "Buildings",
    "ideas": "IDEAS",
    "reduced_order": "AixLib reduced order",
    "iso_13790": "ISO 13790",
    "mpc": "mpc",
}
NOT_APPLICABLE = "-"


def _cell(spec: ParameterSpec, library: str) -> str:
    mapping = spec.mapping(library)
    if mapping.handled:
        return "taken another way"
    if mapping.name is None:
        return NOT_APPLICABLE
    notes = []
    if mapping.when_given:
        notes.append("when given")
    if mapping.variants:
        notes.append(", ".join(mapping.variants) + " variant")
    return f"`{mapping.name}`" + (f" ({'; '.join(notes)})" if notes else "")


def _default(spec: ParameterSpec) -> str:
    if spec.computed:
        return "computed"
    return "library default" if spec.default is None else f"`{spec.default}`"


def _row(spec: ParameterSpec) -> str:
    name = f"`{spec.name}`" + (f", `{spec.short_name}`" if spec.short_name else "")
    description = (spec.description or "").replace("|", "\\|").replace("\n", " ")
    if spec.numerical:
        description = f"*numerical.* {description}"
    if spec.deprecated:
        description = f"*deprecated: {spec.deprecated}.* {description}"
    cells = [NOT_APPLICABLE] * len(COLUMNS) if not spec.render else [_cell(spec, library) for library in COLUMNS]
    return f"| {name} | {description} | {_default(spec)} | " + " | ".join(cells) + " |"


def render_parameters() -> str:
    classes_of: dict[type, list[str]] = {}
    for class_name, model in PARAMETERS.items():
        classes_of.setdefault(model, []).append(class_name)
    lines = [
        "# Parameters",
        "",
        "Every parameter is optional: a parameter left out of the YAML is not written to the Modelica "
        'model, so the library\'s own default applies ("library default" below), unless trano has a '
        "default of its own. The library columns give the Modelica name the parameter is rendered under; "
        "a dash means the library does not take it (a value given anyway is ignored with a warning). "
        "*numerical* marks simulation settings rather than physical properties. The page is generated "
        "from `trano/data_models/parameters.yaml`.",
        "",
    ]
    for model, class_names in classes_of.items():
        specs = parameter_specs(model)
        lines += [
            f"## {model.__name__.removesuffix('_')}",
            "",
            "Elements: " + ", ".join(f"`{name.lower()}`" for name in class_names),
            "",
            "| Parameter | Description | Default | " + " | ".join(COLUMNS.values()) + " |",
            "|---|---|---|" + "---|" * len(COLUMNS),
            *(_row(spec) for spec in specs.values()),
            "",
        ]
    return "\n".join(lines)


def write_parameters() -> None:
    Path(__file__).parents[2].joinpath("docs/reference/parameters.md").write_text(render_parameters())


if __name__ == "__main__":
    write_parameters()
