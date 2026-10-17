import numpy as np

from two_link_robot_simulation import dynamics
from two_link_robot_simulation.robot import Link, Robot

robot = Robot(
    [
        Link(1, 0, 1),
        Link(1, 0, 1),
    ]
)


def test_gravity_zero():
    original_g = dynamics.g

    try:
        dynamics.g = 0

        q = np.array([0.5, 0.7])
        gravity = dynamics.Gravity(robot, q)

        assert np.allclose(gravity, [0.0, 0.0])
    finally:
        dynamics.g = original_g


def test_gravity_vertical():
    q = np.array([np.pi / 2, 0.0])

    gravity = dynamics.Gravity(robot, q)

    assert np.allclose(gravity, [0.0, 0.0], atol=1e-10)


def test_forward_dynamics_zero():
    original_g = dynamics.g

    try:
        dynamics.g = 0

        q = np.array([0.5, 0.7])
        q_dot = np.zeros(2)
        tau = np.zeros(2)

        q_ddot = dynamics.ForwardDynamics(
            robot,
            q,
            q_dot,
            tau,
        )

        assert np.allclose(q_ddot, [0.0, 0.0])
    finally:
        dynamics.g = original_g
