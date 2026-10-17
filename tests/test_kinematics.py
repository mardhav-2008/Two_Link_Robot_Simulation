import numpy as np

from two_link_robot_simulation.kinematics import (
    ForwardKinematics,
    InverseKinematics,
)
from two_link_robot_simulation.robot import Link, Robot

robot = Robot(
    [
        Link(1, 0, 1),
        Link(1, 0, 1),
    ]
)


def test_forward_kinematics():
    q = np.array([0.0, 0.0])

    joint, end = ForwardKinematics(robot, q)

    assert np.allclose(joint, (1.0, 0.0))
    assert np.allclose(end, (2.0, 0.0))


def test_inverse_kinematics():
    target = np.array([1.0, 1.0])

    q = InverseKinematics(robot, target)

    assert q is not None

    _, end = ForwardKinematics(robot, q)

    assert np.allclose(end, target)


def test_inverse_kinematics_unreachable():
    target = np.array([3.0, 0.0])

    q = InverseKinematics(robot, target)

    assert q is None
