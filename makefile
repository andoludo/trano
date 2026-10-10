.PHONY: install linting tests simulate all

PYTHON := python3.10



# ── Linting ────────────────────────────────────────────────────────────────────
linting:
	uv run ruff format .
	uv run ruff check --fix --show-fixes --exit-non-zero-on-fix
	uv run mypy


# ── Tests ──────────────────────────────────────────────────────────────────────
tests:
	uv run pytest -m "not simulate and not bestest"

simulate:
	uv run pytest -m simulate

# ── Composite ──────────────────────────────────────────────────────────────────
all: install linting tests
bestest:
	uv run python -m validation.bestest run --workers 2
	uv run pytest -m bestest
