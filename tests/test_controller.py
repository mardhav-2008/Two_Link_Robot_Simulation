import numpy as np

from two_link_robot_simulation.controller import PDController


def test_zero_error():
    controller = PDController(30, 30)

    q = np.array([1.0, 2.0])
    q_dot = np.array([0.5, -0.5])

    tau = controller.torque(q, q_dot, q, q_dot)

    assert np.allclose(tau, [0.0, 0.0])


def test_position_error():
    controller = PDController(10, 0)

    q = np.array([0.0, 0.0])
    q_d = np.array([1.0, -2.0])

    tau = controller.torque(
        q,
        np.zeros(2),
        q_d,
        np.zeros(2),
    )

    assert np.allclose(tau, [10.0, -20.0])


def test_velocity_error():
    controller = PDController(0, 10)

    q_dot = np.array([1.0, -2.0])
    q_d_dot = np.array([3.0, 1.0])

    tau = controller.torque(
        np.zeros(2),
        q_dot,
        np.zeros(2),
        q_d_dot,
    )

    assert np.allclose(tau, [20.0, 30.0])


def test_position_error_takes_shortest_path():
    controller = PDController(10, 0)

    q = np.array([3.0, 0.0])
    q_d = np.array([-3.0, 0.0])

    tau = controller.torque(q, np.zeros(2), q_d, np.zeros(2))

    # -3 - 3 = -6 rad the long way, 2*pi - 6 = 0.283 rad the short way.
    assert np.allclose(tau, [10 * (2 * np.pi - 6.0), 0.0])


def test_wrap_angle_range():
    from two_link_robot_simulation.controller import wrap_angle

    angles = np.linspace(-20, 20, 401)
    wrapped = wrap_angle(angles)

    assert np.all(wrapped >= -np.pi) and np.all(wrapped < np.pi)
    assert np.allclose(np.sin(wrapped), np.sin(angles))
    assert np.allclose(np.cos(wrapped), np.cos(angles))


def test_feedforward_matches_inverse_dynamics_on_reference():
    from two_link_robot_simulation import dynamics
    from two_link_robot_simulation.controller import FEEDFORWARD, control_torque
    from two_link_robot_simulation.robot import Link, Robot

    robot = Robot([Link(), Link()])
    controller = PDController(50, 20)

    q = np.array([0.2, 0.5])
    q_dot = np.array([0.1, -0.3])
    q_ddot = np.array([1.0, -2.0])

    # On the reference the PD part is zero, so the torque is pure model.
    tau = control_torque(robot, controller, q, q_dot, q, q_dot, q_ddot, FEEDFORWARD)

    assert np.allclose(tau, dynamics.InverseDynamics(robot, q, q_dot, q_ddot))


def test_unknown_mode_rejected():
    import pytest

    from two_link_robot_simulation.controller import control_torque
    from two_link_robot_simulation.robot import Link, Robot

    with pytest.raises(ValueError):
        control_torque(
            Robot([Link(), Link()]),
            PDController(1, 1),
            np.zeros(2),
            np.zeros(2),
            np.zeros(2),
            np.zeros(2),
            np.zeros(2),
            "nonsense",
        )
