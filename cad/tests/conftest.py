"""pytest setup: the project on the path; the assembled model built once per session for the slow tests."""
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


def pytest_addoption(parser):
    parser.addoption("--build", action="store_true", help="run the tests that build the model (minutes)")


def pytest_collection_modifyitems(config, items):
    if not config.getoption("--build"):
        skip = pytest.mark.skip(reason="needs --build (builds the model, minutes)")
        for it in items:
            if "build" in it.keywords:
                it.add_marker(skip)


@pytest.fixture(scope="session")
def model():
    import velvet.hull
    from velvet.assembly import assemble
    velvet.hull.FRESH = True                                    # never the step cache: the tests build what the code says, from scratch
    return assemble()
