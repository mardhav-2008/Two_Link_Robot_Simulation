import numpy as np

from two_link_robot_simulation.controller import PDController
from two_link_robot_simulation.integrator import MAX_FRAME_DT, step
from two_link_robot_simulation.robot import Link, Robot


def default_robot():
    return Robot([Link(), Link()])


def run_frames(kp, kd, frame_dt, frames, q_d=(np.pi / 2, 0.0)):
    robot = default_robot()
    controller = PDController(kp, kd)

    q = np.zeros(2)
    q_dot = np.zeros(2)
    q_d = np.array(q_d)
    q_d_dot = np.zeros(2)

    peak = 0.0

    for _ in range(frames):
        q, q_dot, tau = step(robot, controller, q, q_dot, q_d, q_d_dot, frame_dt)
        peak = max(peak, np.max(np.abs(tau)))

    return q, peak


def test_high_gains_stay_stable_at_60_fps():
    # Plain Euler at dt = 1/60 diverges for Kd above ~40 on this robot.
    q, peak = run_frames(100, 100, 1 / 60, 600)

    assert np.allclose(q, [np.pi / 2, 0.0], atol=1e-2)
    assert peak < 300


def test_frame_time_spike_does_not_blow_up():
    # A stalled frame (window drag, slow first frame) must not inject a huge dt.
    q, peak = run_frames(30, 30, 0.5, 300)

    assert np.all(np.isfinite(q))
    assert peak < 300


def test_frame_dt_is_clamped():
    assert MAX_FRAME_DT <= 0.05
