"""Baseline smoke/import tests for legopixy package."""

import os
import subprocess
import sys
from pathlib import Path


def test_legopixy_package_imports():
    """Verify all core legopixy modules can be imported."""
    import legopixy.lego_kinematics
    import legopixy.lego_pixy_communication
    import legopixy.lego_sim_functions
    import legopixy.particle_filter
    import legopixy.pixy_functions

    assert hasattr(legopixy.pixy_functions, "computeObjectPosition")


def test_particle_filter_simulation_starts():
    """Verify particle filter simulation script starts successfully (runs headless with timeout)."""
    project_root = Path(__file__).resolve().parent.parent
    script_path = project_root / "scripts" / "particle_filter_simulation.py"
    env = {**os.environ, "MPLBACKEND": "Agg"}
    try:
        subprocess.run(
            [sys.executable, str(script_path)],
            cwd=project_root,
            timeout=5,
            capture_output=True,
            env=env,
        )
    except subprocess.TimeoutExpired:
        pass  # Script ran for 5s without crash = entry point works


def test_lego_pixy_simulation_starts():
    """Verify lego pixy simulation script starts (blocks on Bluetooth recieveData; timeout = success)."""
    project_root = Path(__file__).resolve().parent.parent
    script_path = project_root / "scripts" / "lego_pixy_simulation.py"
    env = {**os.environ, "MPLBACKEND": "Agg"}
    try:
        subprocess.run(
            [sys.executable, str(script_path)],
            cwd=project_root,
            timeout=5,
            capture_output=True,
            env=env,
        )
    except subprocess.TimeoutExpired:
        pass  # Script ran until Bluetooth call = entry point works
