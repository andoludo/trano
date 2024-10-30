import nox
from nox import Session

nox.options.reuse_existing_virtualenvs = True


@nox.session(python=["3.10"])
def install(session: Session) -> None:
    groups = ["main", "dev", "docs"]
    session.run(
        "poetry",
        "install",
        "--no-root",
        "--sync",
        f"--only={','.join(groups)}",
        external=True,
    )


@nox.session(python=["3.10"])
def linting(session: Session) -> None:
    session.run("poetry", "run", "mypy")
    session.run(
        "pre-commit", "run", "--all-files", "--config", ".pre-commit-config.yaml"
    )
