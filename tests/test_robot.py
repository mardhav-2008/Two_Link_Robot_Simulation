import numpy as np

from two_link_robot_simulation.robot import Joint, Link, Robot


def test_link_properties():
    link = Link(length=2, angle=0.5, mass=4)

    assert link.length == 2
    assert link.angle == 0.5
    assert link.mass == 4
    assert link.com == 1
    assert np.isclose(link.inertia, 4 * 2**2 / 12)


def test_joint_angle():
    link1 = Link(2, 0.5, 4)
    link2 = Link(1, 1.2, 2)

    joint = Joint(link1, link2)

    assert np.isclose(joint.angle, 0.7)


def test_robot():
    link1 = Link(2, 0, 4)
    link2 = Link(1, 0.5, 2)

    robot = Robot([link1, link2])

    assert len(robot.links) == 2
    assert len(robot.joints) == 1
    assert robot.joints[0].link1 is link1
    assert robot.joints[0].link2 is link2
