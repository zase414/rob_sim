import numpy as np
import pytest

from crs_sim import URDF_PATH_SR450, RobotBosch
from crs_sim.puzzles import HOOP_IN_FLANGE


@pytest.fixture
def robot():
    r = RobotBosch(visualize=False, camera=False, speedup=np.inf)
    r.initialize()
    return r


# ctu_bosch_sr450 tests/test_bosch.py
def test_fk_as_ctu_bosch(robot):
    x, y, _, phi = robot.fk([0, 0, 0, 0])
    assert np.isclose(x, np.sum(robot.link_lengths))
    assert np.isclose(y, 0.0) and np.isclose(phi, 0.0)
    for k in (0, -1, 1, 2):
        assert np.isclose(robot.fk([0, 0, 0, np.deg2rad(10 + k * 360)])[-1],
                          np.deg2rad(10))


@pytest.mark.parametrize("with_phi", [False, True])
def test_ik_reaches_fk_pose(robot, with_phi):
    np.random.seed(0)
    for _ in range(300):
        q = np.random.uniform(robot.q_min, robot.q_max)
        x, y, z, phi = robot.fk(q)
        sols = robot.ik(x, y, z, phi) if with_phi else robot.ik_xyz(x, y, z, q[3])
        assert any(np.allclose(q, s) for s in sols)
        for s in sols:
            assert np.allclose(robot.fk(s), (x, y, z, phi))


def test_flange_pose_is_fk(robot):
    rng = np.random.default_rng(2)
    for _ in range(50):
        q = rng.uniform(robot.q_min, robot.q_max)
        x, y, z, phi = robot.fk(q)
        p = robot.flange_pose(q)
        assert np.allclose(p[:3, 3], [x, y, z])
        assert np.allclose(p[:3, 2], [0, 0, -1])          # tool points down
        hoop = (p @ HOOP_IN_FLANGE)[:3, 3]                 # hoop points along phi
        assert np.allclose(hoop[:2] - [x, y], 0.135 * np.array([np.cos(phi), np.sin(phi)]))


def test_urdf_flange_is_flange_pose(robot):
    pin = pytest.importorskip("pinocchio")
    model, _, _ = pin.buildModelsFromUrdf(str(URDF_PATH_SR450),
                                          package_dirs=[str(URDF_PATH_SR450.parent)])
    data = model.createData()
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = rng.uniform(robot.q_min, robot.q_max)
        pin.framesForwardKinematics(model, data, robot.model_q(q))
        assert np.allclose(data.oMf[model.getFrameId("flange")].homogeneous,
                           robot.flange_pose(q), atol=1e-9)


def test_same_hoop_as_crs97(robot):
    assert np.allclose(robot.hoop_in_flange, HOOP_IN_FLANGE)


def test_moves_like_the_real_one(robot):
    with pytest.raises(AssertionError):
        RobotBosch(visualize=False, camera=False).move_to_q(np.zeros(4))
    with pytest.raises(AssertionError):
        robot.move_to_q(robot.q_max + 0.1)
    q = np.array([0.3, -0.5, -0.1, 1.0])
    robot.move_to_q(q)
    robot.wait_for_motion_stop()
    assert not robot.in_motion()
    assert np.allclose(robot.get_q(), q, atol=1e-4)


def test_real_speed():
    r = RobotBosch(visualize=False, camera=False)
    r.initialize(home=False)
    # default speed of joint A: 10000 IRC*256/ms -> ~0.72 rad/s
    assert np.isclose(r._max_velocity[0], 10000 * 1000 / 256 / 54286.97186604)
