.PHONY: install linting tests simulate all

PYTHON := python3.10



# ── Linting ────────────────────────────────────────────────────────────────────
linting:
	uv run ruff format .
	uv run ruff check --fix --show-fixes --exit-non-zero-on-fix
	uv run mypy


# ── Tests ──────────────────────────────────────────────────────────────────────
tests:
	uv run pytest -m "not simulate"

simulate:
	uv run pytest -m simulate

# ── Composite ──────────────────────────────────────────────────────────────────
all: install linting tests