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


def InverseKinematics(robot: Robot, target: np.ndarray) -> np.ndarray:
    x, y = target

    links = robot.links
    L1 = links[0].length
    L2 = links[1].length

    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2 = np.atan2(np.sqrt(1 - cos_theta2**2), cos_theta2)

    theta1 = np.atan2(y, x) - np.atan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

    return np.array([theta1, theta2])
