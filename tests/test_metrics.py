import numpy as np

from two_link_robot_simulation import metrics


def test_settling_time_finds_first_full_window():
    time = np.arange(0, 3, metrics.DT)
    error = np.where(time < 1.0, 1.0, 0.0)

    assert np.isclose(metrics.settling_time(time, error), 1.0, atol=metrics.DT)


def test_settling_time_window_at_very_end_counts():
    window = int(metrics.SETTLING_WINDOW / metrics.DT)
    time = np.arange(window + 10) * metrics.DT
    error = np.ones_like(time)
    error[10:] = 0.0  # exactly one full window at the end

    assert np.isclose(metrics.settling_time(time, error), time[10])


def test_settling_time_never():
    time = np.arange(0, 2, metrics.DT)

    assert np.isinf(metrics.settling_time(time, np.ones_like(time)))


def test_overshoot_zero_for_monotone_approach():
    positions = np.linspace(metrics.START, metrics.TARGET, 50)

    assert metrics.joint_overshoot(positions, metrics.TARGET) == (0.0, 0.0)


def test_feedforward_tracks_and_settles():
    run = metrics.simulate(60, 50, metrics.TRAJECTORY, "feedforward")
    summary = metrics.summarize(60, 50, run, metrics.TRAJECTORY, "feedforward")

    assert summary["stable"]
    assert summary["tracking_max"] < 0.05


def test_trajectory_lowers_peak_torque_vs_step():
    step = metrics.summarize(100, 85, metrics.simulate(100, 85, metrics.STEP))
    smooth = metrics.summarize(100, 85, metrics.simulate(100, 85))

    assert smooth["peak_torque"] < step["peak_torque"]


def test_output_dir_is_inside_project():
    assert metrics.OUTPUT_DIR.parts[-3:] == ("data", "processed", "controller")
    assert (metrics.PROJECT_ROOT / "pyproject.toml").exists()


def test_normalize_ignores_differences_below_resolution():
    assert np.all(metrics.normalize([0.01, 0.06], resolution=0.5) == 0)
    assert np.allclose(metrics.normalize([1.0, 2.0, 3.0]), [0.0, 0.5, 1.0])


def test_ranking_ties_prefer_lower_gains():
    base = {
        "stable": True,
        "settling_time": 2.0,
        "steady_state_error": 0.0,
        "theta1_overshoot": 0.0,
        "theta2_overshoot": 0.0,
        "peak_torque": 100.0,
        "tracking_rms": 0.0,
    }
    results = [dict(base, kp=50, kd=50), dict(base, kp=10, kd=5)]

    ranking = metrics.rank_controllers(results)

    assert (ranking[0]["kp"], ranking[0]["kd"]) == (10, 5)
