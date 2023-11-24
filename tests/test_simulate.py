from pathlib import Path

import docker
import pytest


@pytest.fixture
def client():
    client = docker.DockerClient(base_url="unix://var/run/docker.sock")
    return client


@pytest.fixture
def container(client):
    container = client.containers.run(
        "openmodelica/openmodelica:v1.22.0-ompython",
        command="tail -f /dev/null",
        volumes=[f"{str(Path(__file__).parent)}:/model"],
        detach=True
    )
    container.exec_run(cmd="omc /model/install_package.mos")
    yield container
    container.stop()
    container.remove()


def test_simulate(container):
    results = container.exec_run(cmd="omc /model/simulate.mos")
    assert results.exit_code == 0
