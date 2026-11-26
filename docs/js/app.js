// Loads the results exported by scripts/export_web_results.py and fills the
// results, comparison and Pareto sections. If the JSON files are missing the
// page still works: the interactive arm does not depend on them.

const MODE_LABELS = {
    pd_gravity: "PD + gravity",
    feedforward: "Feedforward",
};

const CONFIG_LABELS = {
    "step|pd_gravity": "Step target, PD + gravity",
    "trajectory|pd_gravity": "Trajectory, PD + gravity",
    "trajectory|feedforward": "Trajectory, feedforward",
};

const MODE_NOTES = {
    pd_gravity:
        "400 gain pairs (Kp, Kd from 5 to 100), each moving the arm from " +
        "horizontal to [90°, 45°] along a 2.5 s planned trajectory. The PD " +
        "terms do all the work of moving the arm, so low gains are too slow " +
        "to settle within the 6 s run.",
    feedforward:
        "Same 400 runs with inverse-dynamics feedforward. The model supplies " +
        "the torque the motion needs, so every gain pair settles and the " +
        "differences between them are tiny (note the axis scale below). " +
        "Ties go to the lowest gains.",
};

const cache = {};

function format(value, digits, unit = "") {
    if (value === null || value === undefined) {
        return "did not settle";
    }
    return `${Number(value).toFixed(digits)}${unit}`;
}

function maxOvershoot(row) {
    return Math.max(row.theta1_overshoot ?? 0, row.theta2_overshoot ?? 0);
}

async function fetchJson(path) {
    if (!(path in cache)) {
        const response = await fetch(path);
        if (!response.ok) {
            throw new Error(`${path}: HTTP ${response.status}`);
        }
        cache[path] = await response.json();
    }
    return cache[path];
}

function showTableMessage(tbody, columns, message) {
    tbody.innerHTML = "";
    const row = tbody.insertRow();
    const cell = row.insertCell();
    cell.colSpan = columns;
    cell.textContent = message;
}

function fillRow(tbody, values) {
    const row = tbody.insertRow();
    values.forEach(value => {
        row.insertCell().textContent = value;
    });
}

async function showMode(mode) {
    document.querySelectorAll(".mode-switch button").forEach(button => {
        button.setAttribute("aria-pressed", String(button.dataset.mode === mode));
    });

    document.getElementById("modeNote").textContent = MODE_NOTES[mode];

    const tbody = document.getElementById("resultsTable");

    try {
        const [ranking, summary, pareto] = await Promise.all([
            fetchJson(`data/${mode}/controller_ranking.json`),
            fetchJson("data/summary.json"),
            fetchJson(`data/${mode}/pareto_frontier.json`),
        ]);

        const stats = summary.modes.find(entry => entry.mode === mode);
        const best = ranking[0];

        document.getElementById("bestController").textContent =
            best ? `Kp ${best.kp}, Kd ${best.kd}` : "none settled";
        document.getElementById("settlingTime").textContent =
            best ? format(best.settling_time, 2, " s") : "--";
        document.getElementById("overshoot").textContent =
            best ? format(maxOvershoot(best), 1, "%") : "--";
        document.getElementById("peakTorque").textContent =
            best ? format(best.peak_torque, 1, " N·m") : "--";
        document.getElementById("settledRuns").textContent =
            stats ? `${stats.settled} of ${stats.runs}` : "--";

        tbody.innerHTML = "";
        ranking.slice(0, 10).forEach(row => {
            fillRow(tbody, [
                row.rank,
                row.kp,
                row.kd,
                format(row.settling_time, 2, " s"),
                format(maxOvershoot(row), 2, "%"),
                format(row.peak_torque, 1, " N·m"),
                format(row.tracking_rms, 4, " rad"),
            ]);
        });

        if (!ranking.length) {
            showTableMessage(tbody, 7, `No ${MODE_LABELS[mode]} controller settled within the run.`);
        }

        drawPareto(pareto);
    } catch (error) {
        showTableMessage(
            tbody,
            7,
            "Results not found. Run `uv run robot-metrics`, then " +
            "`uv run python scripts/export_web_results.py`.",
        );
        drawPareto([]);
    }
}

async function showComparison() {
    const tbody = document.getElementById("comparisonTable");

    try {
        const rows = await fetchJson("data/configuration_comparison.json");

        tbody.innerHTML = "";
        rows.forEach(row => {
            fillRow(tbody, [
                `${row.kp}, ${row.kd}`,
                CONFIG_LABELS[`${row.reference}|${row.mode}`] ?? `${row.reference}, ${row.mode}`,
                format(row.peak_torque, 1, " N·m"),
                format(row.settling_time, 2, " s"),
                format(row.tracking_rms, 4, " rad"),
            ]);
        });
    } catch (error) {
        showTableMessage(tbody, 5, "Comparison not found. Export the results first.");
    }
}

document.querySelectorAll(".mode-switch button").forEach(button => {
    button.addEventListener("click", () => showMode(button.dataset.mode));
});

showMode("pd_gravity");
showComparison();
