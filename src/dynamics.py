import numpy as np

from robot import Robot

g = 10


def ForwardDynamics(
    robot: Robot, q: np.ndarray, velocity: np.ndarray, tau: np.ndarray
) -> np.ndarray:
    links = robot.links
    theta1_dot, theta2_dot = velocity

    theta1, theta2 = q

    I = np.array([link.inertia for link in links])

    M_11 = (
        I[0]
        + I[1]
        + 1 / 4 * links[0].mass * links[0].length ** 2
        + 1 / 4 * links[1].mass * links[1].length ** 2
        + links[1].mass * links[0].length ** 2
        + links[1].mass * links[0].length * links[1].length * np.cos(theta2)
    )

    M_12 = (
        I[1]
        + 1 / 4 * links[1].mass * links[1].length ** 2
        + 1 / 2 * links[1].mass * links[0].length * links[1].length * np.cos(theta2)
    )

    M_22 = I[1] + 1 / 4 * links[1].mass * links[1].length ** 2

    M = np.array([[M_11, M_12], [M_12, M_22]])

    G_1 = 1 / 2 * links[0].mass * g * links[0].length * np.cos(theta1) + links[
        1
    ].mass * g * (
        links[0].length * np.cos(theta1)
        + 1 / 2 * links[1].length * np.cos(theta1 + theta2)
    )

    G_2 = 1 / 2 * links[1].mass * g * links[1].length * np.cos(theta1 + theta2)

    G = np.array([G_1, G_2])

    h = -1 / 2 * links[1].mass * links[0].length * links[1].length * np.sin(theta2)

    C_1 = h * (2 * theta1_dot * theta2_dot + theta2_dot**2)
    C_2 = -h * theta1_dot**2

    C = np.array([C_1, C_2])

    acceleration = np.linalg.solve(M, tau - C - G)

    return acceleration
