"""Export the experiment CSVs to JSON for the GitHub Pages site (docs/).

Run after `robot-metrics`:

    uv run python scripts/export_web_results.py

Input:  data/processed/controller/<mode>/*.csv and configuration_comparison.csv
Output: docs/data/<mode>/*.json, docs/data/configuration_comparison.json,
        docs/data/summary.json
"""

import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INPUT = ROOT / "data" / "processed" / "controller"
OUTPUT = ROOT / "docs" / "data"

# Must match metrics.PD_GRAVITY / metrics.FEEDFORWARD. Kept as literals so the
# script runs without importing the package.
MODES = ("pd_gravity", "feedforward")

BOOLEAN_COLUMNS = {"stable"}

# Columns the website needs; everything else stays in the CSVs.
PARETO_COLUMNS = (
    "kp",
    "kd",
    "settling_time",
    "steady_state_error",
    "max_overshoot",
    "peak_torque",
    "tracking_rms",
)


def convert(value):
    """CSV text to a JSON-safe value. inf/nan become null (JSON has neither)."""
    value = value.strip()

    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False

    try:
        number = float(value)
    except ValueError:
        return value

    if not math.isfinite(number):
        return None

    return int(number) if number.is_integer() else number


def read_csv(path):
    if not path.exists():
        raise SystemExit(f"Missing {path}. Run `uv run robot-metrics` first.")

    with path.open(newline="", encoding="utf-8") as file:
        rows = []

        for row in csv.DictReader(file):
            cleaned = {}

            for key, value in row.items():
                if key in BOOLEAN_COLUMNS:
                    cleaned[key] = value.strip().lower() == "true"
                else:
                    cleaned[key] = convert(value)

            rows.append(cleaned)

        return rows


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.write("\n")

    print(f"wrote {path.relative_to(ROOT)}")


def export_mode(mode):
    source = INPUT / mode
    target = OUTPUT / mode

    results = read_csv(source / "controller_results.csv")
    ranking = read_csv(source / "controller_ranking.csv")
    pareto = read_csv(source / "pareto_frontier.csv")

    pareto_points = [
        {column: row.get(column) for column in PARETO_COLUMNS}
        for row in pareto
        if row.get("settling_time") is not None
    ]

    save_json(target / "controller_results.json", results)
    save_json(target / "controller_ranking.json", ranking)
    save_json(target / "pareto_frontier.json", pareto_points)

    settled = sum(1 for row in results if row["stable"])

    return {
        "mode": mode,
        "runs": len(results),
        "settled": settled,
        "pareto_points": len(pareto_points),
        "best": ranking[0] if ranking else None,
    }


def main():
    summary = {"modes": [export_mode(mode) for mode in MODES]}

    comparison = read_csv(INPUT / "configuration_comparison.csv")
    save_json(OUTPUT / "configuration_comparison.json", comparison)

    save_json(OUTPUT / "summary.json", summary)

    print()

    for entry in summary["modes"]:
        print(
            f"{entry['mode']:12s} runs={entry['runs']} settled={entry['settled']} "
            f"pareto={entry['pareto_points']}"
        )


if __name__ == "__main__":
    main()
