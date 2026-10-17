import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from . import dynamics
from .controller import PDController
from .robot import Link, Robot

OUTPUT_DIR = Path("data/processed/controller")

# Physics simulation frequency
DT = 1 / 240
SIMULATION_TIME = 5.0

KP_VALUES = np.arange(5, 101, 5)
KD_VALUES = np.arange(5, 101, 5)

ERROR_THRESHOLD = 0.02
SETTLING_WINDOW = 0.5

TARGET = np.array([np.pi / 2, np.pi / 4])


def simulate(kp, kd):
    robot = Robot(
        [
            Link(2, 0, 10),
            Link(1, 0, 5),
        ]
    )

    controller = PDController(kp, kd)

    q = np.array([0.0, 0.0])
    q_dot = np.array([0.0, 0.0])

    q_d = TARGET.copy()
    q_d_dot = np.zeros(2)

    time = np.arange(0, SIMULATION_TIME, DT)

    errors = []
    torques = []
    positions = []

    for _ in time:
        pd_tau = controller.torque(
            q,
            q_dot,
            q_d,
            q_d_dot,
        )

        gravity_tau = dynamics.Gravity(
            robot,
            q,
        )

        tau = pd_tau + gravity_tau

        q_ddot = dynamics.ForwardDynamics(
            robot,
            q,
            q_dot,
            tau,
        )

        q_dot += q_ddot * DT
        q += q_dot * DT

        error = np.linalg.norm(q_d - q)

        errors.append(error)
        torques.append(np.linalg.norm(tau))
        positions.append(q.copy())

    return (
        time,
        np.array(errors),
        np.array(torques),
        np.array(positions),
    )


def settling_time(
    time,
    error,
    threshold=ERROR_THRESHOLD,
):
    window_size = max(
        1,
        int(SETTLING_WINDOW / DT),
    )

    for i in range(len(error) - window_size):
        if np.all(error[i : i + window_size] < threshold):
            return time[i]

    return np.inf


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

    initial_error = np.abs(target - positions[0])

    overshoot_values = []

    for joint in range(2):
        if initial_error[joint] == 0:
            overshoot_values.append(0.0)
            continue

        _error = positions[:, joint] - target[joint]

        # Target is greater than initial
        # position.
        if target[joint] > positions[0, joint]:
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


def run_experiments():

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    results = []

    for kp in KP_VALUES:
        for kd in KD_VALUES:
            (
                time,
                error,
                torque,
                positions,
            ) = simulate(
                kp,
                kd,
            )

            (
                theta1_overshoot,
                theta2_overshoot,
            ) = joint_overshoot(
                positions,
                TARGET,
            )

            results.append(
                {
                    "kp": kp,
                    "kd": kd,
                    "settling_time": settling_time(
                        time,
                        error,
                    ),
                    "steady_state_error": steady_state_error(
                        error,
                    ),
                    "theta1_overshoot": theta1_overshoot,
                    "theta2_overshoot": theta2_overshoot,
                    "peak_torque": peak_torque(
                        torque,
                    ),
                    "stable": is_stable(
                        time,
                        error,
                    ),
                }
            )

    return results


def save_results(results):

    path = OUTPUT_DIR / "controller_results.csv"

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
                "theta1_overshoot",
                "theta2_overshoot",
                "peak_torque",
                "stable",
            ],
        )

        writer.writeheader()
        writer.writerows(results)


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

    def normalize(values):

        minimum = np.min(values)
        maximum = np.max(values)

        if maximum == minimum:
            return np.zeros_like(values)

        return (values - minimum) / (maximum - minimum)

    settling = normalize(settling)

    error = normalize(error)

    overshoot = normalize(overshoot)

    torque = normalize(torque)

    for i, result in enumerate(stable_results):
        result["score"] = (
            0.35 * settling[i]
            + 0.15 * error[i]
            + 0.30 * overshoot[i]
            + 0.20 * torque[i]
        )

    return sorted(
        stable_results,
        key=lambda result: result["score"],
    )


def save_ranking(ranking):

    path = OUTPUT_DIR / "controller_ranking.csv"

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
                }
            )


def plot_kp_stability(results):

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
    plt.ylabel("Stable runs (%)")
    plt.title("Kp vs Stability")
    plt.grid()

    plt.savefig(OUTPUT_DIR / "kp_vs_stability.png")

    plt.close()


def plot_kd_stability(results):

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
    plt.ylabel("Stable runs (%)")
    plt.title("Kd vs Stability")
    plt.grid()

    plt.savefig(OUTPUT_DIR / "kd_vs_stability.png")

    plt.close()


def plot_kp_settling_time(results):

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

    plt.savefig(OUTPUT_DIR / "kp_vs_settling_time.png")

    plt.close()


def plot_kd_settling_time(results):

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

    plt.savefig(OUTPUT_DIR / "kd_vs_settling_time.png")

    plt.close()


def plot_stability_map(results):

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

    plt.imshow(
        stability,
        origin="lower",
        aspect="auto",
        extent=(
            kd_values[0],
            kd_values[-1],
            kp_values[0],
            kp_values[-1],
        ),
    )

    plt.colorbar(label="Stable")

    plt.xlabel("Kd")
    plt.ylabel("Kp")
    plt.title("Kp vs Kd Stability Map")

    plt.savefig(OUTPUT_DIR / "kp_kd_stability.png")

    plt.close()


def plot_overshoot(results):

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

    plt.savefig(OUTPUT_DIR / "kp_vs_overshoot.png")

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
            )

            strictly_better = (
                other["settling_time"] < candidate["settling_time"]
                or other["steady_state_error"] < candidate["steady_state_error"]
                or other["max_overshoot"] < candidate["max_overshoot"]
                or other["peak_torque"] < candidate["peak_torque"]
            )

            if no_worse and strictly_better:
                dominated = True
                break

        if not dominated:
            pareto.append(candidate)

    return pareto


def save_pareto_frontier(pareto):

    path = OUTPUT_DIR / "pareto_frontier.csv"

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
                }
            )


def plot_pareto_frontier(
    results,
    pareto,
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

    plt.savefig(OUTPUT_DIR / "pareto_frontier.png")

    plt.close()


def main():

    print("Running controller experiments...")

    results = run_experiments()

    save_results(results)

    ranking = rank_controllers(results)

    save_ranking(ranking)

    pareto = find_pareto_frontier(results)

    save_pareto_frontier(pareto)

    plot_pareto_frontier(results, pareto)

    plot_kp_stability(results)

    plot_kd_stability(results)

    plot_kp_settling_time(results)

    plot_kd_settling_time(results)

    plot_stability_map(results)

    plot_overshoot(results)

    print(f"Results saved to {OUTPUT_DIR}")

    print("\nTop 10 controllers:")

    for result in ranking[:10]:
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
            f"Torque={result['peak_torque']:.2f}"
        )

    print("\nPareto-optimal controllers:")

    for result in sorted(
        pareto,
        key=lambda result: result["settling_time"],
    ):
        print(
            f"Kp={result['kp']:3d}, "
            f"Kd={result['kd']:3d}, "
            f"Ts={result['settling_time']:.3f}s, "
            f"Overshoot={result['max_overshoot']:.2f}%, "
            f"Error={result['steady_state_error']:.5f}, "
            f"Torque={result['peak_torque']:.2f}"
        )


if __name__ == "__main__":
    main()
