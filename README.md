# rob_sim

Simulator of the **CRS97** and **Mitsubishi RV-6S** arms for the ROB1 hoop and
wire puzzle task. It has the same interface as the real drivers
([`ctu_crs`](https://pypi.org/project/ctu-crs/),
[`ctu_mitsubishi`](https://pypi.org/project/ctu-mitsubishi/)), so the same
script runs on the real robot and in the simulator without changes.

## Setup

```bash
git clone https://github.com/zase414/rob_sim.git && cd rob_sim
./setup_venv.sh            # creates .venv, installs the simulator + pytest
```

## How to use

Write `src/main.py` as normal real-robot code, starting with
`from ctu_crs import CRS97`. Do not import `crs_sim` in it.

1. Connected to the real robot (on the lab PC, where `ctu_crs` is installed):

   ```bash
   python src/main.py
   ```

2. In the simulator, with the viewer opening in your browser:

   ```bash
   ./sim.sh src/main.py
   ```

Options for the simulated scene go before `src/main.py`, for example
`./sim.sh --puzzle C --seed 7 src/main.py`. `./sim.sh --help` lists them all.
At the end the simulator prints a verdict: contacts with the puzzle and how far
down the wire the hoop got.

On the Mitsubishi, write `from ctu_mitsubishi import Rv6s` / `robot = Rv6s()`
instead and run it the same way. The simulator gives you whichever robot your
script creates.

Other entry points:

```bash
./start.sh                 # interactive console with a simulated `robot`
./start.sh -m pytest       # run the tests
```

The full API reference (camera, puzzles, contacts, viewer, how faithful the
simulator is to the real robots) is in [README.AGENTS.md](README.AGENTS.md).

## Credits

This is a fork of [Amanak421/crs_sim](https://github.com/Amanak421/crs_sim) by
Jakub Andrlík, cut down to a clean starting point for students.

The puzzle CAD models come from
[pilcsimo/ROB1-2025-hw](https://github.com/pilcsimo/ROB1-2025-hw) (GPL-3.0).
They are not included here; they are downloaded to `~/.cache/crs_sim/` the
first time you use them.

## License

The kinematics and robot parameters are taken from `ctu_crs` (BSD 3-Clause,
Copyright (c) 2023, CTURobotics), see [LICENSE-ctu_crs](LICENSE-ctu_crs). The
RV-6S kinematics are taken from `ctu_mitsubishi` (BSD 3-Clause, Copyright (c)
2023, CTURobotics), see [LICENSE-ctu_mitsubishi](LICENSE-ctu_mitsubishi).
