"""CLI for the BESTEST validation suite.

Examples
--------
::

    uv run python -m validation.bestest run-all
    uv run python -m validation.bestest run-case 600FF
    uv run python -m validation.bestest report
"""
from __future__ import annotations

from pathlib import Path

import typer

from validation.bestest.harness import CACHE_ROOT, docker_available, run_all, run_case
from validation.bestest.report import load_reference_bands, write_report
from validation.bestest.spec.parameters import CASES, KPIResults

app = typer.Typer(no_args_is_help=True, add_completion=False)

REPORT_DIR: Path = Path(__file__).resolve().parents[1] / "bestest" / "_reports"


def _require_docker() -> None:
    if not docker_available():
        typer.secho(
            "Docker daemon not reachable; BESTEST requires OpenModelica via Docker.",
            fg=typer.colors.RED,
            err=True,
        )
        raise typer.Exit(code=2)


@app.command("list")
def list_cases() -> None:
    """List the BESTEST cases the suite covers."""
    for case_id in sorted(CASES):
        params = CASES[case_id]
        typer.echo(
            f"  {case_id:5}  base={params.base:11}  "
            f"FF={params.free_float}  heat={params.has_heating}  cool={params.has_cooling}"
        )


@app.command("run-case")
def run_case_cmd(case_id: str, force: bool = False) -> None:
    """Run a single BESTEST case and print its KPIs."""
    _require_docker()
    if case_id not in CASES:
        typer.secho(f"unknown case {case_id!r}; use `list`", fg=typer.colors.RED, err=True)
        raise typer.Exit(code=2)
    result = run_case(case_id, force=force)
    typer.echo(result.model_dump_json(indent=2))


@app.command("run-all")
def run_all_cmd(force: bool = False) -> None:
    """Run every case sequentially and write the report."""
    _require_docker()
    results = run_all(force=force)
    bands = load_reference_bands()
    write_report(results, bands, REPORT_DIR)
    typer.echo(f"Wrote report to {REPORT_DIR}")


@app.command("report")
def report_cmd() -> None:
    """Re-render the report from cached KPI results without running simulations."""
    results: dict[str, KPIResults] = {}
    for case_id in sorted(CASES):
        kpis_file = CACHE_ROOT / case_id / "kpis.json"
        if kpis_file.exists():
            results[case_id] = KPIResults.model_validate_json(kpis_file.read_text())
    if not results:
        typer.secho("no cached results; run `run-all` first", fg=typer.colors.YELLOW)
        raise typer.Exit(code=1)
    bands = load_reference_bands()
    write_report(results, bands, REPORT_DIR)
    typer.echo(f"Wrote report to {REPORT_DIR}  ({len(results)} cases)")


if __name__ == "__main__":
    app()
