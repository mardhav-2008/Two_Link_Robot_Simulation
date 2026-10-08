import numpy as np

from robot import Robot

PI = np.pi
BASE = (0, 0)


def ForwardKinematics(
    robot: Robot, q: np.ndarray
) -> tuple[tuple[float, float], tuple[float, float]]:
    theta1, theta2 = q
    L1, L2 = [link.length for link in robot.links]

    x_1 = L1 * np.cos(theta1)
    y_1 = L1 * np.sin(theta1)

    x_end = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y_end = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)

    return ((x_1, y_1), (x_end, y_end))
