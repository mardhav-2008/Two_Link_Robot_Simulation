import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # files only; works without a display

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np

from .controller import FEEDFORWARD, PD_GRAVITY, PDController, wrap_angle
from .integrator import step
from .robot import Link, Robot
from .trajectory import plan

# Anchored to the project root, so results land in the same place whatever
# directory the command is run from.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = PROJECT_ROOT / "data" / "processed" / "controller"

# Physics simulation frequency
DT = 1 / 240
# Long enough for a ~2.5 s planned move plus the 0.5 s settling window.
SIMULATION_TIME = 6.0

KP_VALUES = np.arange(5, 101, 5)
KD_VALUES = np.arange(5, 101, 5)

ERROR_THRESHOLD = 0.02
SETTLING_WINDOW = 0.5

TARGET = np.array([np.pi / 2, np.pi / 4])
START = np.zeros(2)

# Reference used by the main sweep: a quintic move (default) or the old
# instantaneous step, kept for comparison.
TRAJECTORY = "trajectory"
STEP = "step"

# Joint limits for planning on the experiment robot (2 m / 10 kg and
# 1 m / 5 kg links). It is ~3x heavier than the interactive arm, so the
# limits are lower: the pi/2 rad move takes ~2.5 s. Tested values:
#   1.5 / 2.5 -> T 2.0 s, feedforward peak torque 336 N m
#   1.2 / 1.5 -> T 2.5 s, feedforward peak torque 294 N m  (chosen)
#   1.0 / 1.2 -> T 3.0 s, feedforward peak torque 272 N m
# The step reference peaks at 392 N m (Kp=100); 221 N m of any value is
# the static gravity load of the arm held horizontal.
EXPERIMENT_MAX_VELOCITY = 1.2  # rad/s
EXPERIMENT_MAX_ACCELERATION = 1.5  # rad/s^2

# Configurations compared in the reference/control-law experiment.
COMPARISON_CONFIGS = [
    (STEP, PD_GRAVITY),
    (TRAJECTORY, PD_GRAVITY),
    (TRAJECTORY, FEEDFORWARD),
]
COMPARISON_GAINS = [(30, 30), (60, 50), (100, 85)]

# Move durations swept for the peak torque vs duration plot.
DURATIONS = np.arange(0.5, 4.01, 0.25)
DURATION_GAINS = (60, 50)


def experiment_robot():
    return Robot(
        [
            Link(2, 0, 10),
            Link(1, 0, 5),
        ]
    )


def experiment_trajectory(duration=None):
    return plan(
        START,
        TARGET,
        0.0,
        duration=duration,
        max_velocity=EXPERIMENT_MAX_VELOCITY,
        max_acceleration=EXPERIMENT_MAX_ACCELERATION,
    )


def simulate(kp, kd, reference=TRAJECTORY, mode=PD_GRAVITY, duration=None):
    """Run one closed-loop move from START to TARGET.

    Returns a dict of time series:
      time, error (to the final target), tracking_error (to the reference),
      torque (norm), torques (per joint), positions, references, move_duration.
    """
    robot = experiment_robot()
    controller = PDController(kp, kd)

    q = START.copy()
    q_dot = np.zeros(2)

    q_d = TARGET.copy()
    q_d_dot = np.zeros(2)

    trajectory = experiment_trajectory(duration) if reference == TRAJECTORY else None
    move_duration = trajectory.duration if trajectory is not None else 0.0

    time = np.arange(0, SIMULATION_TIME, DT)

    errors = []
    tracking_errors = []
    torques = []
    positions = []
    references = []

    for t in time:
        if trajectory is not None:
            q_ref = trajectory.sample(t)[0]
        else:
            q_ref = q_d

        # Single physics step per sample: DT = 1/240 is well inside the
        # stability limit for this robot (Kd < ~300).
        q, q_dot, tau = step(
            robot,
            controller,
            q,
            q_dot,
            q_d,
            q_d_dot,
            DT,
            physics_dt=DT,
            trajectory=trajectory,
            t=t,
            mode=mode,
        )

        errors.append(np.linalg.norm(wrap_angle(q_d - q)))
        tracking_errors.append(np.linalg.norm(wrap_angle(q_ref - q)))
        torques.append(tau)
        positions.append(q.copy())
        references.append(np.array(q_ref, dtype=float))

    torques = np.array(torques)

    # Each sample is the state at the END of its step.
    return {
        "time": time + DT,
        "error": np.array(errors),
        "tracking_error": np.array(tracking_errors),
        "torque": np.linalg.norm(torques, axis=1),
        "torques": torques,
        "positions": np.array(positions),
        "references": np.array(references),
        "move_duration": move_duration,
    }


def tracking_rms(tracking_error):
    return float(np.sqrt(np.mean(tracking_error**2)))


def tracking_max(tracking_error):
    return float(np.max(tracking_error))


def summarize(kp, kd, run, reference=TRAJECTORY, mode=PD_GRAVITY):
    time = run["time"]
    error = run["error"]

    theta1_overshoot, theta2_overshoot = joint_overshoot(run["positions"], TARGET)

    settle = settling_time(time, error)

    return {
        "kp": kp,
        "kd": kd,
        "reference": reference,
        "mode": mode,
        "move_duration": run["move_duration"],
        "settling_time": settle,
        # Time the arm needs after the planned move ends. For a step
        # reference this equals settling_time.
        "settle_after_move": max(0.0, settle - run["move_duration"]),
        "steady_state_error": steady_state_error(error),
        "theta1_overshoot": theta1_overshoot,
        "theta2_overshoot": theta2_overshoot,
        "peak_torque": peak_torque(run["torque"]),
        "peak_tau1": float(np.max(np.abs(run["torques"][:, 0]))),
        "peak_tau2": float(np.max(np.abs(run["torques"][:, 1]))),
        "tracking_rms": tracking_rms(run["tracking_error"]),
        "tracking_max": tracking_max(run["tracking_error"]),
        "stable": is_stable(time, error),
    }


def settling_time(
    time,
    error,
    threshold=ERROR_THRESHOLD,
):
    """First time after which the error stays below threshold for a full
    SETTLING_WINDOW. np.inf if that never happens within the run."""
    window_size = max(
        1,
        int(SETTLING_WINDOW / DT),
    )

    if len(error) < window_size:
        return np.inf

    inside = (np.asarray(error) < threshold).astype(int)

    # Number of in-band samples in each window, all windows at once.
    counts = np.convolve(inside, np.ones(window_size, dtype=int), mode="valid")

    starts = np.flatnonzero(counts == window_size)

    if starts.size == 0:
        return np.inf

    return time[starts[0]]


def steady_state_error(error):
    samples = max(
        1,
        int(SETTLING_WINDOW / DT),
    )

    return np.mean(error[-samples:])


def joint_overshoot(
    positions,
    target,
):
    """
    Calculate overshoot percentage
    for each joint.

    Overshoot is measured relative
    to the initial position error.
    """

    initial_error = np.abs(target - START)

    overshoot_values = []

    for joint in range(2):
        if initial_error[joint] == 0:
            overshoot_values.append(0.0)
            continue

        _error = positions[:, joint] - target[joint]

        # Target is greater than initial
        # position.
        if target[joint] > START[joint]:
            peak = np.max(positions[:, joint])

            overshoot_amount = max(
                0.0,
                peak - target[joint],
            )

        # Target is smaller than initial
        # position.
        else:
            peak = np.min(positions[:, joint])

            overshoot_amount = max(
                0.0,
                target[joint] - peak,
            )

        overshoot_percentage = (overshoot_amount / initial_error[joint]) * 100

        overshoot_values.append(overshoot_percentage)

    return (
        overshoot_values[0],
        overshoot_values[1],
    )


def peak_torque(torque):
    return np.max(torque)


def is_stable(time, error):
    """
    A run is considered stable if it
    satisfies the settling-time criterion.
    """

    return np.isfinite(
        settling_time(
            time,
            error,
        )
    )


def run_experiments(reference=TRAJECTORY, mode=PD_GRAVITY):

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    results = []

    for kp in KP_VALUES:
        for kd in KD_VALUES:
            run = simulate(kp, kd, reference, mode)
            results.append(summarize(kp, kd, run, reference, mode))

    return results


RESULT_FIELDS = [
    "kp",
    "kd",
    "reference",
    "mode",
    "move_duration",
    "settling_time",
    "settle_after_move",
    "steady_state_error",
    "theta1_overshoot",
    "theta2_overshoot",
    "peak_torque",
    "peak_tau1",
    "peak_tau2",
    "tracking_rms",
    "tracking_max",
    "stable",
]


def save_results(results, output_dir=OUTPUT_DIR):

    path = output_dir / "controller_results.csv"

    with path.open(
        "w",
        newline="",
    ) as file:
        writer = csv.DictWriter(file, fieldnames=RESULT_FIELDS)

        writer.writeheader()
        writer.writerows(results)


# Smallest difference in each metric that is treated as real. Below this,
# min-max normalisation would stretch numerical noise (e.g. 0.01 % vs
# 0.06 % overshoot) to the full 0-1 range and decide the ranking on it.
RESOLUTION = {
    "settling_time": 2 * DT,  # s
    "steady_state_error": 1e-3,  # rad
    "overshoot": 0.5,  # %
    "peak_torque": 1.0,  # N m
    "tracking_rms": 1e-3,  # rad
}


def normalize(values, resolution=0.0):
    """Scale to 0-1 (0 = best). All zeros when the spread is below resolution."""
    values = np.asarray(values, dtype=float)

    minimum = np.min(values)
    maximum = np.max(values)

    if maximum - minimum <= resolution:
        return np.zeros_like(values)

    return (values - minimum) / (maximum - minimum)


def rank_controllers(results):

    stable_results = [result for result in results if result["stable"]]

    if not stable_results:
        return []

    settling = np.array([result["settling_time"] for result in stable_results])

    error = np.array([result["steady_state_error"] for result in stable_results])

    overshoot = np.array(
        [
            max(
                result["theta1_overshoot"],
                result["theta2_overshoot"],
            )
            for result in stable_results
        ]
    )

    torque = np.array([result["peak_torque"] for result in stable_results])

    tracking = np.array([result["tracking_rms"] for result in stable_results])

    settling = normalize(settling, RESOLUTION["settling_time"])

    error = normalize(error, RESOLUTION["steady_state_error"])

    overshoot = normalize(overshoot, RESOLUTION["overshoot"])

    torque = normalize(torque, RESOLUTION["peak_torque"])

    tracking = normalize(tracking, RESOLUTION["tracking_rms"])

    # Weights sum to 1. Tracking is included because, with a planned
    # reference, following the path is part of the job, not just arriving.
    for i, result in enumerate(stable_results):
        result["score"] = (
            0.30 * settling[i]
            + 0.15 * error[i]
            + 0.20 * overshoot[i]
            + 0.15 * torque[i]
            + 0.20 * tracking[i]
        )

    # Ties (common once differences fall below RESOLUTION) go to the lower
    # gains: they amplify sensor noise less and need less actuator bandwidth.
    return sorted(
        stable_results,
        key=lambda result: (round(result["score"], 9), result["kp"] + result["kd"]),
    )


def save_ranking(ranking, output_dir=OUTPUT_DIR):

    path = output_dir / "controller_ranking.csv"

    with path.open(
        "w",
        newline="",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "rank",
                "kp",
                "kd",
                "score",
                "settling_time",
                "steady_state_error",
                "theta1_overshoot",
                "theta2_overshoot",
                "peak_torque",
                "tracking_rms",
            ],
        )

        writer.writeheader()

        for rank, result in enumerate(
            ranking,
            start=1,
        ):
            writer.writerow(
                {
                    "rank": rank,
                    "kp": result["kp"],
                    "kd": result["kd"],
                    "score": result["score"],
                    "settling_time": result["settling_time"],
                    "steady_state_error": result["steady_state_error"],
                    "theta1_overshoot": result["theta1_overshoot"],
                    "theta2_overshoot": result["theta2_overshoot"],
                    "peak_torque": result["peak_torque"],
                    "tracking_rms": result["tracking_rms"],
                }
            )


def plot_kp_stability(results, output_dir=OUTPUT_DIR):

    kp_values = sorted({result["kp"] for result in results})

    stability = []

    for kp in kp_values:
        values = [result["stable"] for result in results if result["kp"] == kp]

        stability.append(np.mean(values) * 100)

    plt.figure()

    plt.plot(
        kp_values,
        stability,
        marker="o",
    )

    plt.xlabel("Kp")
    plt.ylabel("Runs settled within the simulation (%)")
    plt.title("Kp vs share of settled runs")
    plt.grid()

    plt.savefig(output_dir / "kp_vs_stability.png")

    plt.close()


def plot_kd_stability(results, output_dir=OUTPUT_DIR):

    kd_values = sorted({result["kd"] for result in results})

    stability = []

    for kd in kd_values:
        values = [result["stable"] for result in results if result["kd"] == kd]

        stability.append(np.mean(values) * 100)

    plt.figure()

    plt.plot(
        kd_values,
        stability,
        marker="o",
    )

    plt.xlabel("Kd")
    plt.ylabel("Runs settled within the simulation (%)")
    plt.title("Kd vs share of settled runs")
    plt.grid()

    plt.savefig(output_dir / "kd_vs_stability.png")

    plt.close()


def plot_kp_settling_time(results, output_dir=OUTPUT_DIR):

    kp_values = sorted({result["kp"] for result in results})

    settling_times = []

    for kp in kp_values:
        values = [
            result["settling_time"]
            for result in results
            if (result["kp"] == kp and np.isfinite(result["settling_time"]))
        ]

        settling_times.append(np.mean(values) if values else np.nan)

    plt.figure()

    plt.plot(
        kp_values,
        settling_times,
        marker="o",
    )

    plt.xlabel("Kp")
    plt.ylabel("Settling time (s)")
    plt.title("Kp vs Settling Time")
    plt.grid()

    plt.savefig(output_dir / "kp_vs_settling_time.png")

    plt.close()


def plot_kd_settling_time(results, output_dir=OUTPUT_DIR):

    kd_values = sorted({result["kd"] for result in results})

    settling_times = []

    for kd in kd_values:
        values = [
            result["settling_time"]
            for result in results
            if (result["kd"] == kd and np.isfinite(result["settling_time"]))
        ]

        settling_times.append(np.mean(values) if values else np.nan)

    plt.figure()

    plt.plot(
        kd_values,
        settling_times,
        marker="o",
    )

    plt.xlabel("Kd")
    plt.ylabel("Settling time (s)")
    plt.title("Kd vs Settling Time")
    plt.grid()

    plt.savefig(output_dir / "kd_vs_settling_time.png")

    plt.close()


def plot_stability_map(results, output_dir=OUTPUT_DIR):

    kp_values = sorted({result["kp"] for result in results})

    kd_values = sorted({result["kd"] for result in results})

    stability = np.zeros(
        (
            len(kp_values),
            len(kd_values),
        )
    )

    for i, kp in enumerate(kp_values):
        for j, kd in enumerate(kd_values):
            result = next(
                result
                for result in results
                if (result["kp"] == kp and result["kd"] == kd)
            )

            stability[i, j] = int(result["stable"])

    plt.figure()

    # Extent goes half a grid step past the first/last value so each cell
    # is centred on its (Kd, Kp) pair.
    kd_step = kd_values[1] - kd_values[0] if len(kd_values) > 1 else 1
    kp_step = kp_values[1] - kp_values[0] if len(kp_values) > 1 else 1

    plt.imshow(
        stability,
        origin="lower",
        aspect="auto",
        extent=(
            kd_values[0] - kd_step / 2,
            kd_values[-1] + kd_step / 2,
            kp_values[0] - kp_step / 2,
            kp_values[-1] + kp_step / 2,
        ),
    )

    plt.colorbar(label="1 = settled, 0 = did not settle")

    plt.xlabel("Kd")
    plt.ylabel("Kp")
    plt.title(f"Kp vs Kd settling map ({SIMULATION_TIME:.0f} s runs)")

    plt.savefig(output_dir / "kp_kd_stability.png")

    plt.close()


def plot_overshoot(results, output_dir=OUTPUT_DIR):

    kp_values = sorted({result["kp"] for result in results})

    theta1 = []
    theta2 = []

    for kp in kp_values:
        values = [
            result for result in results if (result["kp"] == kp and result["stable"])
        ]

        if values:
            theta1.append(np.mean([result["theta1_overshoot"] for result in values]))

            theta2.append(np.mean([result["theta2_overshoot"] for result in values]))

        else:
            theta1.append(np.nan)
            theta2.append(np.nan)

    plt.figure()

    plt.plot(
        kp_values,
        theta1,
        marker="o",
        label="θ1",
    )

    plt.plot(
        kp_values,
        theta2,
        marker="o",
        label="θ2",
    )

    plt.xlabel("Kp")
    plt.ylabel("Overshoot (%)")
    plt.title("Kp vs Joint Overshoot")

    plt.legend()
    plt.grid()

    plt.savefig(output_dir / "kp_vs_overshoot.png")

    plt.close()


def find_pareto_frontier(results):
    stable_results = [result for result in results if result["stable"]]

    for result in stable_results:
        result["max_overshoot"] = max(
            result["theta1_overshoot"],
            result["theta2_overshoot"],
        )

    pareto = []

    for candidate in stable_results:
        dominated = False

        for other in stable_results:
            if candidate is other:
                continue

            no_worse = (
                other["settling_time"] <= candidate["settling_time"]
                and other["steady_state_error"] <= candidate["steady_state_error"]
                and other["max_overshoot"] <= candidate["max_overshoot"]
                and other["peak_torque"] <= candidate["peak_torque"]
                and other["tracking_rms"] <= candidate["tracking_rms"]
            )

            strictly_better = (
                other["settling_time"] < candidate["settling_time"]
                or other["steady_state_error"] < candidate["steady_state_error"]
                or other["max_overshoot"] < candidate["max_overshoot"]
                or other["peak_torque"] < candidate["peak_torque"]
                or other["tracking_rms"] < candidate["tracking_rms"]
            )

            if no_worse and strictly_better:
                dominated = True
                break

        if not dominated:
            pareto.append(candidate)

    return pareto


def save_pareto_frontier(pareto, output_dir=OUTPUT_DIR):

    path = output_dir / "pareto_frontier.csv"

    with path.open(
        "w",
        newline="",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "kp",
                "kd",
                "settling_time",
                "steady_state_error",
                "max_overshoot",
                "peak_torque",
                "tracking_rms",
            ],
        )

        writer.writeheader()

        for result in pareto:
            writer.writerow(
                {
                    "kp": result["kp"],
                    "kd": result["kd"],
                    "settling_time": result["settling_time"],
                    "steady_state_error": result["steady_state_error"],
                    "max_overshoot": result["max_overshoot"],
                    "peak_torque": result["peak_torque"],
                    "tracking_rms": result["tracking_rms"],
                }
            )


def plot_pareto_frontier(
    results,
    pareto,
    output_dir=OUTPUT_DIR,
):

    plt.figure()

    stable = [result for result in results if result["stable"]]

    plt.scatter(
        [
            max(
                result["theta1_overshoot"],
                result["theta2_overshoot"],
            )
            for result in stable
        ],
        [result["settling_time"] for result in stable],
        alpha=0.4,
        label="Stable controllers",
    )

    plt.scatter(
        [result["max_overshoot"] for result in pareto],
        [result["settling_time"] for result in pareto],
        marker="o",
        label="Pareto frontier",
    )

    for result in pareto:
        plt.annotate(
            f"({result['kp']}, {result['kd']})",
            (
                result["max_overshoot"],
                result["settling_time"],
            ),
            xytext=(5, 5),
            textcoords="offset points",
        )

    plt.xlabel("Maximum Joint Overshoot (%)")

    plt.ylabel("Settling Time (s)")

    plt.title("Controller Pareto Frontier")

    plt.grid()
    plt.legend()

    plt.savefig(output_dir / "pareto_frontier.png")

    plt.close()


def compare_configurations():
    """Step vs trajectory reference, PD+gravity vs feedforward."""
    rows = []

    for kp, kd in COMPARISON_GAINS:
        for reference, mode in COMPARISON_CONFIGS:
            run = simulate(kp, kd, reference, mode)
            rows.append(summarize(kp, kd, run, reference, mode))

    path = OUTPUT_DIR / "configuration_comparison.csv"

    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    return rows


def config_label(reference, mode):
    if reference == STEP:
        return "step + PD/gravity"

    if mode == FEEDFORWARD:
        return "trajectory + feedforward"

    return "trajectory + PD/gravity"


def plot_configuration_comparison(rows):
    labels = [f"Kp={kp}, Kd={kd}" for kp, kd in COMPARISON_GAINS]
    x = np.arange(len(labels))
    width = 0.8 / len(COMPARISON_CONFIGS)

    figure, axes = plt.subplots(1, 3, figsize=(16, 5))

    for metric, axis, title in (
        ("peak_torque", axes[0], "Peak torque (N m)"),
        ("settling_time", axes[1], "Settling time (s)"),
        ("tracking_rms", axes[2], "Tracking RMS error (rad)"),
    ):
        for i, (reference, mode) in enumerate(COMPARISON_CONFIGS):
            values = [
                next(
                    row[metric]
                    for row in rows
                    if row["kp"] == kp
                    and row["kd"] == kd
                    and row["reference"] == reference
                    and row["mode"] == mode
                )
                for kp, kd in COMPARISON_GAINS
            ]

            # A run that never settled has settling_time = inf; draw no bar
            # for it and mark it instead.
            heights = [value if np.isfinite(value) else np.nan for value in values]
            positions = x + i * width

            axis.bar(positions, heights, width, label=config_label(reference, mode))

            for position, value in zip(positions, values):
                if not np.isfinite(value):
                    axis.annotate(
                        "did not\nsettle",
                        (position, 0),
                        ha="center",
                        va="bottom",
                        fontsize=7,
                    )

        axis.set_xticks(x + width * (len(COMPARISON_CONFIGS) - 1) / 2, labels)
        axis.set_title(title)
        axis.grid(axis="y")

    axes[0].legend()
    figure.tight_layout()
    figure.savefig(OUTPUT_DIR / "configuration_comparison.png")
    plt.close(figure)


def plot_tracking(kp, kd):
    """Planned vs actual joint angles and joint torques over time."""
    figure, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

    for reference, mode in COMPARISON_CONFIGS:
        run = simulate(kp, kd, reference, mode)
        label = config_label(reference, mode)

        for joint in range(2):
            axes[0, joint].plot(run["time"], run["positions"][:, joint], label=label)
            axes[1, joint].plot(run["time"], run["torques"][:, joint], label=label)

    reference_run = simulate(kp, kd, TRAJECTORY, PD_GRAVITY)

    for joint in range(2):
        axes[0, joint].plot(
            reference_run["time"],
            reference_run["references"][:, joint],
            "k--",
            label="planned",
        )
        axes[0, joint].axhline(TARGET[joint], color="grey", linewidth=0.8)
        axes[0, joint].set_title(f"θ{joint + 1} (rad)")
        axes[1, joint].set_title(f"τ{joint + 1} (N m)")
        axes[1, joint].set_xlabel("Time (s)")

        for row in range(2):
            axes[row, joint].grid()

    axes[0, 0].legend()
    figure.suptitle(f"Trajectory tracking, Kp={kp}, Kd={kd}")
    figure.tight_layout()
    figure.savefig(OUTPUT_DIR / "trajectory_tracking.png")
    plt.close(figure)


def plot_peak_torque_vs_duration(kp=DURATION_GAINS[0], kd=DURATION_GAINS[1]):
    """Shorter moves need more torque: shows the speed/effort trade-off."""
    rows = []

    for mode in (PD_GRAVITY, FEEDFORWARD):
        for duration in DURATIONS:
            run = simulate(kp, kd, TRAJECTORY, mode, duration=duration)
            row = summarize(kp, kd, run, TRAJECTORY, mode)
            rows.append(row)

    step_peak = summarize(kp, kd, simulate(kp, kd, STEP, PD_GRAVITY), STEP)[
        "peak_torque"
    ]

    plt.figure()

    for mode in (PD_GRAVITY, FEEDFORWARD):
        plt.plot(
            DURATIONS,
            [row["peak_torque"] for row in rows if row["mode"] == mode],
            marker="o",
            label=config_label(TRAJECTORY, mode),
        )

    plt.axhline(step_peak, color="red", linestyle="--", label="step reference")

    plt.xlabel("Move duration (s)")
    plt.ylabel("Peak torque (N m)")
    plt.title(f"Peak torque vs move duration (Kp={kp}, Kd={kd})")
    plt.grid()
    plt.legend()

    plt.savefig(OUTPUT_DIR / "peak_torque_vs_duration.png")
    plt.close()

    return rows


def run_sweep(mode, output_dir):
    """Full Kp/Kd sweep for one control law; writes CSVs and plots."""
    output_dir.mkdir(parents=True, exist_ok=True)

    results = run_experiments(TRAJECTORY, mode)

    save_results(results, output_dir)

    ranking = rank_controllers(results)
    save_ranking(ranking, output_dir)

    pareto = find_pareto_frontier(results)
    save_pareto_frontier(pareto, output_dir)
    plot_pareto_frontier(results, pareto, output_dir)

    plot_kp_stability(results, output_dir)
    plot_kd_stability(results, output_dir)
    plot_kp_settling_time(results, output_dir)
    plot_kd_settling_time(results, output_dir)
    plot_stability_map(results, output_dir)
    plot_overshoot(results, output_dir)

    return results, ranking, pareto


def print_ranking(title, ranking, count=10):
    print(f"\n{title}")

    for result in ranking[:count]:
        overshoot = max(
            result["theta1_overshoot"],
            result["theta2_overshoot"],
        )

        print(
            f"Kp={result['kp']:3d}, "
            f"Kd={result['kd']:3d}, "
            f"Score={result['score']:.4f}, "
            f"Ts={result['settling_time']:.3f}s, "
            f"Overshoot={overshoot:.2f}%, "
            f"Torque={result['peak_torque']:.2f}, "
            f"Tracking RMS={result['tracking_rms']:.4f}"
        )


def print_pareto(title, pareto):
    print(f"\n{title}")

    for result in sorted(pareto, key=lambda result: result["settling_time"]):
        print(
            f"Kp={result['kp']:3d}, "
            f"Kd={result['kd']:3d}, "
            f"Ts={result['settling_time']:.3f}s, "
            f"Overshoot={result['max_overshoot']:.2f}%, "
            f"Error={result['steady_state_error']:.5f}, "
            f"Torque={result['peak_torque']:.2f}, "
            f"Tracking RMS={result['tracking_rms']:.4f}"
        )


def main():

    print("Running controller experiments...")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    sweeps = {}

    for mode in (PD_GRAVITY, FEEDFORWARD):
        print(f"  sweep: {mode}")
        sweeps[mode] = run_sweep(mode, OUTPUT_DIR / mode)

    print("  comparison runs")

    comparison = compare_configurations()
    plot_configuration_comparison(comparison)

    plot_peak_torque_vs_duration()

    best = sweeps[PD_GRAVITY][1]
    if best:
        plot_tracking(best[0]["kp"], best[0]["kd"])

    print(f"Results saved to {OUTPUT_DIR}")

    print("\nReference / control-law comparison:")

    for row in comparison:
        print(
            f"Kp={row['kp']:3d}, Kd={row['kd']:3d}, "
            f"{config_label(row['reference'], row['mode']):26s} "
            f"Torque={row['peak_torque']:7.2f}, "
            f"Ts={row['settling_time']:.3f}s, "
            f"Tracking RMS={row['tracking_rms']:.4f}"
        )

    for mode, (results, ranking, pareto) in sweeps.items():
        settled = sum(bool(result["stable"]) for result in results)

        print(f"\n=== {mode}: {settled}/{len(results)} gain pairs settled ===")

        print_ranking(f"Top 10 controllers ({mode}):", ranking)
        print_pareto(f"Pareto-optimal controllers ({mode}):", pareto)


if __name__ == "__main__":
    main()
