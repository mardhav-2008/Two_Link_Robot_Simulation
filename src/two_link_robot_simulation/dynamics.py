import numpy as np

from .robot import Robot

g = 9.81


def MassMatrix(robot: Robot, q: np.ndarray) -> np.ndarray:
    links = robot.links
    _theta1, theta2 = q

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

    return np.array([[M_11, M_12], [M_12, M_22]])


def Coriolis(robot: Robot, q: np.ndarray, velocity: np.ndarray) -> np.ndarray:
    links = robot.links
    theta1_dot, theta2_dot = velocity
    _theta1, theta2 = q

    h = -1 / 2 * links[1].mass * links[0].length * links[1].length * np.sin(theta2)

    C_1 = h * (2 * theta1_dot * theta2_dot + theta2_dot**2)
    C_2 = -h * theta1_dot**2

    return np.array([C_1, C_2])


def ForwardDynamics(
    robot: Robot, q: np.ndarray, velocity: np.ndarray, tau: np.ndarray
) -> np.ndarray:
    M = MassMatrix(robot, q)
    C = Coriolis(robot, q, velocity)
    G = Gravity(robot, q)

    return np.linalg.solve(M, tau - C - G)


def InverseDynamics(
    robot: Robot, q: np.ndarray, velocity: np.ndarray, acceleration: np.ndarray
) -> np.ndarray:
    """Joint torques needed to produce the given acceleration: M q_ddot + C + G."""
    return (
        MassMatrix(robot, q) @ acceleration
        + Coriolis(robot, q, velocity)
        + Gravity(robot, q)
    )


def Gravity(robot: Robot, q: np.ndarray) -> np.ndarray:
    links = robot.links
    theta1, theta2 = q

    G_1 = 1 / 2 * links[0].mass * g * links[0].length * np.cos(theta1) + links[
        1
    ].mass * g * (
        links[0].length * np.cos(theta1)
        + 1 / 2 * links[1].length * np.cos(theta1 + theta2)
    )

    G_2 = 1 / 2 * links[1].mass * g * links[1].length * np.cos(theta1 + theta2)

    return np.array([G_1, G_2])
