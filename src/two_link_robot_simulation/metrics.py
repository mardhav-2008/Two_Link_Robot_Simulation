import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from . import dynamics
from .controller import PDController
from .robot import Link, Robot


OUTPUT_DIR = Path("data/processed/controller")

DT = 1 / 60
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

        gravity_tau = dynamics.Gravity(robot, q)

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


def settling_time(time, error, threshold=ERROR_THRESHOLD):
    window_size = max(1, int(SETTLING_WINDOW / DT))

    for i in range(len(error) - window_size):
        if np.all(error[i : i + window_size] < threshold):
            return time[i]

    return np.inf


def steady_state_error(error):
    samples = max(1, int(SETTLING_WINDOW / DT))

    return np.mean(error[-samples:])


def joint_overshoot(positions, target):
    """
    Calculate overshoot percentage for each joint.

    Overshoot is measured relative to the initial position error.
    """

    initial_error = np.abs(target - positions[0])

    overshoot_values = []

    for joint in range(2):
        if initial_error[joint] == 0:
            overshoot_values.append(0.0)
            continue

        error = positions[:, joint] - target[joint]

        # Look for motion past the target.
        if target[joint] > positions[0, joint]:
            peak = np.max(positions[:, joint])
            overshoot_amount = max(0.0, peak - target[joint])
        else:
            peak = np.min(positions[:, joint])
            overshoot_amount = max(0.0, target[joint] - peak)

        overshoot_percentage = (overshoot_amount / initial_error[joint]) * 100

        overshoot_values.append(overshoot_percentage)

    return overshoot_values[0], overshoot_values[1]


def peak_torque(torque):
    return np.max(torque)


def is_stable(time, error):
    return np.isfinite(settling_time(time, error))


def run_experiments():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    results = []

    for kp in KP_VALUES:
        for kd in KD_VALUES:
            (
                time,
                error,
                torque,
                positions,
            ) = simulate(kp, kd)

            theta1_overshoot, theta2_overshoot = joint_overshoot(
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
                    "peak_torque": peak_torque(torque),
                    "stable": is_stable(
                        time,
                        error,
                    ),
                }
            )

    return results


def save_results(results):
    path = OUTPUT_DIR / "controller_results.csv"

    with path.open("w", newline="") as file:
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


def plot_kp_stability(results):
    kp_values = sorted(set(result["kp"] for result in results))

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
    kd_values = sorted(set(result["kd"] for result in results))

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
    kp_values = sorted(set(result["kp"] for result in results))

    settling_times = []

    for kp in kp_values:
        values = [
            result["settling_time"]
            for result in results
            if result["kp"] == kp and np.isfinite(result["settling_time"])
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
    kd_values = sorted(set(result["kd"] for result in results))

    settling_times = []

    for kd in kd_values:
        values = [
            result["settling_time"]
            for result in results
            if result["kd"] == kd and np.isfinite(result["settling_time"])
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
    kp_values = sorted(set(result["kp"] for result in results))

    kd_values = sorted(set(result["kd"] for result in results))

    stability = np.zeros((len(kp_values), len(kd_values)))

    for i, kp in enumerate(kp_values):
        for j, kd in enumerate(kd_values):
            result = next(
                result
                for result in results
                if result["kp"] == kp and result["kd"] == kd
            )

            stability[i, j] = int(result["stable"])

    plt.figure()

    plt.imshow(
        stability,
        origin="lower",
        aspect="auto",
        extent=[
            kd_values[0],
            kd_values[-1],
            kp_values[0],
            kp_values[-1],
        ],
    )

    plt.colorbar(label="Stable")

    plt.xlabel("Kd")
    plt.ylabel("Kp")
    plt.title("Kp vs Kd Stability Map")

    plt.savefig(OUTPUT_DIR / "kp_kd_stability.png")

    plt.close()


def plot_overshoot(results):
    kp_values = sorted(set(result["kp"] for result in results))

    theta1 = []
    theta2 = []

    for kp in kp_values:
        values = [
            result for result in results if result["kp"] == kp and result["stable"]
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


def main():
    print("Running controller experiments...")

    results = run_experiments()

    save_results(results)

    plot_kp_stability(results)
    plot_kd_stability(results)
    plot_kp_settling_time(results)
    plot_kd_settling_time(results)
    plot_stability_map(results)
    plot_overshoot(results)

    print(f"Results saved to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
