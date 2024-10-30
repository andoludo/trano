from dotenv import load_dotenv

from trano.main import report

load_dotenv()


def test_multizone_free_float() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        "./multizone_free_float.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="Buildings",
        ),
    )


def test_simulate_model_buildings() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        "./hello_world.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="Buildings",
        ),
    )


def test_simulate_model_ideas() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        "./hello_world.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="IDEAS",
        ),
    )


def test_report() -> None:
    from trano.simulate.simulate import SimulationLibraryOptions

    report(
        "./multizone_free_float.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="Buildings",
        ),
    )
