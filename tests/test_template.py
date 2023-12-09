from pathlib import Path

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel
import networkx as nx

from neosim.model import Space, Window


def test_template_buildings_free_float_single_zone(buildings_free_float_single_zone):
    model_ = buildings_free_float_single_zone.model()


def test_template_buildings_free_float_two_zones(buildings_free_float_two_zones):
    model_ = buildings_free_float_two_zones.model()


def test_template_buildings_free_float_three_zones(buildings_free_float_three_zones):
    model_ = buildings_free_float_three_zones.model()
