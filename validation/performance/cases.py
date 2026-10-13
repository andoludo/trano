"""Models and option profiles of the performance benchmark."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from trano.data_models.conversion import convert_network
from trano.elements.control import BoilerControl
from trano.elements.library.library import Library
from trano.performance import PerformanceOptions
from trano.topology import Network
from validation.bestest.harness import reset_element_names

REPOSITORY = Path(__file__).resolve().parents[2]
TEST_MODELS = REPOSITORY.joinpath("tests", "models")
EXAMPLES = REPOSITORY.joinpath("examples")
LIBRARIES = ("Buildings", "IDEAS")
# Boiler controls with tank temperature checks; the "simple" variant (heat pumps) has none.
HYSTERESIS_VARIANTS = ("default", "complex")


@dataclass(frozen=True)
class BenchmarkModel:
    """A YAML model of the benchmark; ``envelope_only`` models have no hydronic system."""

    name: str
    path: Path
    envelope_only: bool = False
    slow: bool = False  # several minutes per simulated week: only run on request


MODELS: tuple[BenchmarkModel, ...] = (
    BenchmarkModel("two_zones", TEST_MODELS.joinpath("two_zones.yaml"), envelope_only=True),
    BenchmarkModel("single_zone_hydronic", TEST_MODELS.joinpath("single_zone_hydronic.yaml")),
    BenchmarkModel("three_zones_hydronic", TEST_MODELS.joinpath("three_zones_hydronic.yaml")),
    BenchmarkModel("house_infiltration_boiler", TEST_MODELS.joinpath("house_infiltration_boiler.yaml")),
    BenchmarkModel("residential_after_renovation", EXAMPLES.joinpath("residential_after_renovation.yaml")),
    BenchmarkModel("house_complex", TEST_MODELS.joinpath("house_complex.yaml"), slow=True),
)


@dataclass(frozen=True)
class Profile:
    """Performance options applied to every model; ``boiler_hysteresis`` [K] goes to the boiler controls."""

    name: str
    description: str
    performance: PerformanceOptions = field(default_factory=PerformanceOptions.disabled)
    boiler_hysteresis: float | None = None


PROFILES: dict[str, Profile] = {
    profile.name: profile
    for profile in (
        Profile("baseline", "the detailed model: the performance options off"),
        Profile(
            "fast",
            "ideal actuators and steady-state sensors (trano's default)",
            PerformanceOptions(ideal_actuators=True, steady_state_sensors=True),
        ),
        Profile("hysteresis", "1 K hysteresis on the boiler tank temperature checks", boiler_hysteresis=1.0),
        Profile(
            "fast_hysteresis",
            "fast and hysteresis together",
            PerformanceOptions(ideal_actuators=True, steady_state_sensors=True),
            boiler_hysteresis=1.0,
        ),
    )
}


@dataclass(frozen=True)
class BenchmarkCase:
    model: BenchmarkModel
    library: str
    profile: Profile

    @property
    def id(self) -> str:
        return f"{self.model.name}_{self.library}_{self.profile.name}"

    @property
    def baseline(self) -> BenchmarkCase:
        return BenchmarkCase(self.model, self.library, PROFILES["baseline"])

    def network(self) -> Network:
        reset_element_names()
        network = convert_network(
            self.model.name,
            self.model.path,
            library=Library.from_configuration(self.library),
            performance=self.profile.performance,
        )
        if self.profile.boiler_hysteresis is not None:
            for node in network.graph.nodes:
                if isinstance(node, BoilerControl) and node.variant in HYSTERESIS_VARIANTS and node.parameters:
                    node.parameters.temperature_hysteresis = self.profile.boiler_hysteresis  # type: ignore[attr-defined]
        return network


def select_cases(
    models: list[str] | None, libraries: list[str] | None, profiles: list[str] | None, include_slow: bool = False
) -> list[BenchmarkCase]:
    """The cases of the selected models, libraries and profiles (all when None; slow models on request)."""
    selected_models = [
        model for model in MODELS if (model.name in models if models else include_slow or not model.slow)
    ]
    unknown = set(models or []) - {model.name for model in MODELS}
    if unknown:
        raise ValueError(f"Unknown models {sorted(unknown)}; available: {[model.name for model in MODELS]}")
    return [
        BenchmarkCase(model, library, PROFILES[profile])
        for model in selected_models
        for library in libraries or LIBRARIES
        for profile in profiles or ["baseline", "fast"]
        if not (model.envelope_only and profile != "baseline")
    ]
