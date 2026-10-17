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
