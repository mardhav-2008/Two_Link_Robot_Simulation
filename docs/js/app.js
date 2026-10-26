// Loads the exported Python results.
// If the JSON files are not present yet, the page still shows the
// interactive robot instead of completely breaking.

async function loadResults() {
    try {
        const response = await fetch("data/controller_results.json");

        if (!response.ok) {
            throw new Error("controller_results.json not found");
        }

        const results = await response.json();

        const rows = document.getElementById("resultsTable");
        rows.innerHTML = "";

        results.forEach(result => {
            const row = document.createElement("tr");

            row.innerHTML = `
                <td>${result.controller}</td>
                <td>${result.settling_time}</td>
                <td>${result.overshoot}</td>
                <td>${result.stability}</td>
            `;

            rows.appendChild(row);
        });

        if (results.length > 0) {
            const best = results[0];

            document.getElementById("bestController").textContent =
                best.controller;
            document.getElementById("settlingTime").textContent =
                best.settling_time;
            document.getElementById("overshoot").textContent =
                best.overshoot;
        }
    } catch (error) {
        document.getElementById("resultsTable").innerHTML = `
            <tr>
                <td colspan="4">
                    Results will appear here after the Python results
                    are exported to docs/data/.
                </td>
            </tr>
        `;
    }
}

async function loadPareto() {
    try {
        const response = await fetch("data/pareto_frontier.json");

        if (!response.ok) {
            throw new Error("pareto data not found");
        }

        const points = await response.json();
        drawPareto(points);
    } catch (error) {
        drawPareto([]);
    }
}

loadResults();
loadPareto();
