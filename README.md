# LegoPixy

[![CI](https://github.com/jprevc/LegoPixy/actions/workflows/ci.yml/badge.svg)](https://github.com/jprevc/LegoPixy/actions/workflows/ci.yml)

Lego Pixy particle filter localization and simulation for LEGO Mindstorms EV3 robots with Pixy camera.

## Authors

- **Jošt Prevc**
- **Miha Mubi**

## Overview

LegoPixy provides particle-filter-based robot localization using a Pixy camera for object detection. The library includes:

- **Particle filter** – Monte Carlo localization with sensor fusion
- **Lego kinematics** – Wheel-to-pose kinematics for differential-drive robots
- **Pixy integration** – Object position computation from camera data
- **Simulation** – Desktop simulations with matplotlib visualization

## Demo

![Particle filtering demo](docs/demo.gif)

## Architecture

```
src/legopixy/          # Reusable library
├── particle_filter.py       # Particle filter localization
├── lego_kinematics.py       # Robot motion model
├── lego_sim_functions.py    # Drawing and simulation helpers
├── lego_pixy_communication.py  # EV3/Pixy communication
└── pixy_functions.py        # Camera coordinate transforms

scripts/               # Runnable simulations
├── particle_filter_simulation.py   # Offline particle filter demo
└── lego_pixy_simulation.py         # Live EV3 + Pixy visualization

hardware/ev3/          # EV3-specific code (excluded from CI)
└── ev3_block_det.py
```

## Setup

Requires Python 3.10+. Uses [uv](https://docs.astral.sh/uv/) for dependency management.

```bash
# Install uv (if needed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install dependencies
uv sync
```

## Run

### Particle filter simulation

Offline particle filter demo with matplotlib visualization:

```bash
uv run python scripts/particle_filter_simulation.py
```

### Lego Pixy simulation

Live visualization driven by real EV3 robot data (requires Bluetooth connection to the robot):

```bash
uv run python scripts/lego_pixy_simulation.py
```

> **Note:** Edit the MAC address in the script to match your EV3 device.

## Tooling

```bash
# Lint
uv run ruff check .

# Format
uv run black .

# Run all pre-commit hooks
uv run pre-commit run -a

# Tests
uv run pytest
```

## Contributing

1. Fork the repo and create a feature branch.
2. Run `uv sync` and ensure `uv run pre-commit run -a` passes.
3. Add tests for new behavior.
4. Open a pull request.

## Roadmap

- [ ] CLI entry points for simulations (`legopixy-sim`, etc.)
- [ ] Reduce duplicated logic between EV3 script and shared modules
- [ ] Optional mypy/pyright static checking
