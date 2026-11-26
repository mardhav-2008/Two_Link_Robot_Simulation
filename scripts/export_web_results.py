from pathlib import Path
import csv
import json


ROOT = Path(__file__).resolve().parents[1]
INPUT = ROOT / "data" / "processed" / "controller"
OUTPUT = ROOT / "docs" / "data"


def convert(value):
    value = value.strip()

    if value.lower() in {"inf", "-inf", "infinity", "-infinity"}:
        return None

    try:
        number = float(value)
        return int(number) if number.is_integer() else number
    except ValueError:
        return value


def read_csv(filename):
    path = INPUT / filename

    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = []

        for row in reader:
            cleaned = {}

            for key, value in row.items():
                if key == "stable":
                    cleaned[key] = value.lower() == "true"
                else:
                    cleaned[key] = convert(value)

            rows.append(cleaned)

        return rows


def save_json(filename, data):
    OUTPUT.mkdir(parents=True, exist_ok=True)

    path = OUTPUT / filename

    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"✓ {path}")


def main():
    results = read_csv("controller_results.csv")
    ranking = read_csv("controller_ranking.csv")
    pareto = read_csv("pareto_frontier.csv")

    save_json("controller_results.json", results)
    save_json("controller_ranking.json", ranking)

    # Only send the useful plotting values to the browser.
    pareto_points = []

    for row in pareto:
        if row["settling_time"] is None:
            continue

        pareto_points.append({
            "kp": row["kp"],
            "kd": row["kd"],
            "settling_time": row["settling_time"],
            "steady_state_error": row["steady_state_error"],
            "max_overshoot": row["max_overshoot"],
            "peak_torque": row["peak_torque"],
        })

    save_json("pareto_frontier.json", pareto_points)

    print()
    print(f"Controller results : {len(results)}")
    print(f"Ranked results     : {len(ranking)}")
    print(f"Pareto points      : {len(pareto_points)}")


if __name__ == "__main__":
    main()
