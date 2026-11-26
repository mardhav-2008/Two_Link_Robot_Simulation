import numpy as np
import pytest

from two_link_robot_simulation.trajectory import (
    MAX_JOINT_ACCELERATION,
    MAX_JOINT_VELOCITY,
    QuinticTrajectory,
    minimum_duration,
    plan,
)


def test_rest_to_rest_boundary_conditions():
    trajectory = QuinticTrajectory(np.zeros(2), np.array([1.0, -0.5]), 0.0, 2.0)

    q, q_dot, q_ddot = trajectory.sample(0.0)
    assert np.allclose(q, [0.0, 0.0])
    assert np.allclose(q_dot, 0.0)
    assert np.allclose(q_ddot, 0.0)

    q, q_dot, q_ddot = trajectory.sample(2.0 - 1e-12)
    assert np.allclose(q, [1.0, -0.5])
    assert np.allclose(q_dot, 0.0, atol=1e-9)
    assert np.allclose(q_ddot, 0.0, atol=1e-6)


def test_holds_end_point_after_finish():
    trajectory = QuinticTrajectory(np.zeros(2), np.ones(2), 1.0, 1.0)

    q, q_dot, q_ddot = trajectory.sample(10.0)

    assert np.allclose(q, 1.0)
    assert np.allclose(q_dot, 0.0)
    assert np.allclose(q_ddot, 0.0)
    assert trajectory.is_finished(2.0)


def test_starts_from_moving_state():
    v0 = np.array([0.5, -1.0])
    a0 = np.array([0.2, 0.3])

    trajectory = QuinticTrajectory(np.zeros(2), np.ones(2), 0.0, 3.0, v0, a0)

    _, q_dot, q_ddot = trajectory.sample(0.0)

    assert np.allclose(q_dot, v0)
    assert np.allclose(q_ddot, a0)


def test_derivatives_match_finite_differences():
    trajectory = QuinticTrajectory(np.zeros(2), np.array([2.0, 1.0]), 0.0, 2.0)
    h = 1e-6

    for t in (0.3, 1.0, 1.7):
        q_minus = trajectory.sample(t - h)[0]
        q_plus = trajectory.sample(t + h)[0]
        _, q_dot, _ = trajectory.sample(t)

        assert np.allclose((q_plus - q_minus) / (2 * h), q_dot, atol=1e-5)


def test_minimum_duration_respects_limits():
    displacement = np.array([np.pi, 0.2])
    duration = minimum_duration(displacement)

    trajectory = QuinticTrajectory(np.zeros(2), displacement, 0.0, duration)
    samples = [trajectory.sample(t) for t in np.linspace(0, duration, 2001)]

    peak_velocity = max(np.max(np.abs(s[1])) for s in samples)
    peak_acceleration = max(np.max(np.abs(s[2])) for s in samples)

    assert peak_velocity <= MAX_JOINT_VELOCITY + 1e-6
    assert peak_acceleration <= MAX_JOINT_ACCELERATION + 1e-6


def test_plan_takes_shortest_way_round():
    trajectory = plan(np.array([3.0, 0.0]), np.array([-3.0, 0.0]), 0.0)

    # 2*pi - 6 = 0.283 rad forward, not 6 rad backward.
    assert np.isclose(trajectory.q_end[0] - 3.0, 2 * np.pi - 6.0)


def test_rejects_non_positive_duration():
    with pytest.raises(ValueError):
        QuinticTrajectory(np.zeros(2), np.ones(2), 0.0, 0.0)
