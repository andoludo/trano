from typing import List

import nox
from nox import Session

nox.options.reuse_existing_virtualenvs = True


def install(session: Session, groups: List[str], root: bool = True) -> None:
    if root:
        groups = ["main", *groups]

    session.run_always(
        "poetry",
        "install",
        "--no-root",
        "--sync",
        f"--only={','.join(groups)}",
        external=True,
    )
    if root:
        session.install(".")


@nox.session(python=["3.10"])
def tests(session: Session) -> None:
    install(session, groups=["dev"])

    session.run("poetry", "run", "mypy")
    session.run(
        "pre-commit", "run", "--all-files", "--config", ".pre-commit-config.yaml"
    )
