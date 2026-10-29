.PHONY: install linting tests compile simulate runtime all

PYTHON := python3.10



# ── Linting ────────────────────────────────────────────────────────────────────
linting:
	uv run ruff format .
	uv run ruff check --fix --show-fixes --exit-non-zero-on-fix
	uv run mypy


# ── Tests ──────────────────────────────────────────────────────────────────────
tests:
	uv run pytest -m "not simulate and not bestest and not compile and not runtime and not systems"

compile:
	uv run pytest -m compile

simulate:
	uv run pytest -m simulate

runtime:
	uv run pytest -m runtime

# ── Composite ──────────────────────────────────────────────────────────────────
all: install linting tests
bestest:
	uv run python -m validation.bestest run --workers 2
	uv run pytest -m bestest
systems:
	uv run python -m validation.systems run --workers 2
	uv run pytest -m systems
