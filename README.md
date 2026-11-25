# rob_sim

Simulator of the **CRS97**, **CRS93**, **Mitsubishi RV-6S** and **Bosch SR450**
arms for the ROB1 hoop and wire puzzle task. It has the same interface as the
real drivers ([`ctu_crs`](https://pypi.org/project/ctu-crs/),
[`ctu_mitsubishi`](https://pypi.org/project/ctu-mitsubishi/),
[`ctu_bosch_sr450`](https://pypi.org/project/ctu-bosch-sr450/)), so the same
script runs on the real robot and in the simulator without changes.

## Setup

```bash
git clone https://github.com/zase414/rob_sim.git && cd rob_sim
./setup_venv.sh            # creates .venv, installs the simulator + pytest
```

Needs Python 3.10 or newer. Everything, the puzzle CAD models included, ships with
the repo, so nothing is downloaded later.

Without the shell scripts (Windows, conda, an existing venv):

```bash
pip install -e .                          # in the environment of your choice
python -m crs_sim.launch src/main.py      # = ./sim.sh src/main.py
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

On the other robots, create them the way you would on the real one and run the
script the same way. The simulator gives you whichever robot your script creates:

| Robot          | In your script                                                      |
| :------------- | :------------------------------------------------------------------ |
| CRS97          | `from ctu_crs import CRS97`, then `robot = CRS97()`                 |
| CRS93          | `from ctu_crs import CRS93`, then `robot = CRS93()`                 |
| Mitsubishi     | `from ctu_mitsubishi import Rv6s`, then `robot = Rv6s()`            |
| Bosch SR450    | `from ctu_bosch_sr450 import RobotBosch`, then `robot = RobotBosch()` |

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

The puzzle, base plate, adapter and hoop CAD models come from
[pilcsimo/ROB1-2025-hw](https://github.com/pilcsimo/ROB1-2025-hw) (GPL-3.0)
and are bundled in `crs_sim/cad/`. The adapter and hoop meshes in
`crs_sim/urdf/meshes/` are converted from them.

## License

GNU General Public License v3.0, see [LICENSE](LICENSE). The project bundles
the GPL-3.0 CAD models above, so it is GPL-3.0 as a whole.

The kinematics and robot parameters are taken from `ctu_crs` (BSD 3-Clause,
Copyright (c) 2023, CTURobotics), see [LICENSE-ctu_crs](LICENSE-ctu_crs). The
RV-6S kinematics are taken from `ctu_mitsubishi` (BSD 3-Clause, Copyright (c)
2023, CTURobotics), see [LICENSE-ctu_mitsubishi](LICENSE-ctu_mitsubishi). The
SR450 kinematics are taken from `ctu_bosch_sr450` (BSD 3-Clause, Copyright (c)
2023, CTURobotics), see [LICENSE-ctu_bosch_sr450](LICENSE-ctu_bosch_sr450).
