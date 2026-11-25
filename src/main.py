"""Your solution. Write it as plain real-robot code: it must not import crs_sim.

    python src/main.py      # on the real robot (ctu_crs / ctu_mitsubishi installed)
    ./sim.sh src/main.py    # in the simulator
"""
from ctu_crs import CRS97  # or: from ctu_mitsubishi import Rv6s


def main():
    robot = CRS97()  # or: robot = Rv6s()
    robot.initialize()
    # TODO: your code here


if __name__ == "__main__":
    main()
