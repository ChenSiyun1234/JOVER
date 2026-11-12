"""Run all physics tests as one guarded batch, with no worker subprocesses."""
import pytest
from sim.resources import wait_for_simulation


@pytest.fixture(scope="session", autouse=True)
def simulation_guard():
    wait_for_simulation()
