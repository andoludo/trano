import tempfile
from pathlib import Path

import docker
import pytest
import jinja2

from tests.conftest import create_mos_file, is_success


@pytest.fixture(scope= "session")
def client():
    client = docker.DockerClient(base_url="unix://var/run/docker.sock")
    return client


@pytest.fixture(scope= "session")
def container(client):
    container = client.containers.run(
        "openmodelica/openmodelica:v1.22.0-ompython",
        command="tail -f /dev/null",
        volumes=[f"{str(Path(__file__).parents[1])}:/neosim"],
        detach=True,
    )
    container.exec_run(cmd="omc /neosim/neosim/library/install_package.mos")
    yield container
    container.stop()
    container.remove()

def test_simulate_buildings_free_float_single_zone(buildings_free_float_single_zone, container):
    with create_mos_file(buildings_free_float_single_zone) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)

def test_simulate_buildings_free_float_two_zones(buildings_free_float_two_zones, container):
    with create_mos_file(buildings_free_float_two_zones) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)

def test_simulate_buildings_free_float_three_zones(buildings_free_float_three_zones, container):
    with create_mos_file(buildings_free_float_three_zones) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)
