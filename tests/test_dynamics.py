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


def test_inverse_dynamics_inverts_forward_dynamics():
    q = np.array([0.3, -1.1])
    q_dot = np.array([0.7, -0.4])
    tau = np.array([5.0, -2.0])

    q_ddot = dynamics.ForwardDynamics(robot, q, q_dot, tau)

    assert np.allclose(dynamics.InverseDynamics(robot, q, q_dot, q_ddot), tau)


def test_mass_matrix_symmetric_positive_definite():
    for theta2 in np.linspace(-np.pi, np.pi, 13):
        M = dynamics.MassMatrix(robot, np.array([0.0, theta2]))

        assert np.allclose(M, M.T)
        assert np.all(np.linalg.eigvalsh(M) > 0)


def test_energy_conserved_without_torque():
    # Free swing under gravity, no damping: total energy must stay constant.
    q = np.array([0.4, 0.8])
    q_dot = np.zeros(2)
    dt = 1e-4

    def energy(q, q_dot):
        kinetic = 0.5 * q_dot @ dynamics.MassMatrix(robot, q) @ q_dot
        l1, l2 = (link.length for link in robot.links)
        m1, m2 = (link.mass for link in robot.links)
        y1 = l1 / 2 * np.sin(q[0])
        y2 = l1 * np.sin(q[0]) + l2 / 2 * np.sin(q[0] + q[1])
        return kinetic + dynamics.g * (m1 * y1 + m2 * y2)

    start = energy(q, q_dot)

    for _ in range(10000):
        q_ddot = dynamics.ForwardDynamics(robot, q, q_dot, np.zeros(2))
        q_dot = q_dot + q_ddot * dt
        q = q + q_dot * dt

    assert abs(energy(q, q_dot) - start) < 1e-2 * abs(start)
