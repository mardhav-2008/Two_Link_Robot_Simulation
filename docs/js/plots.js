// Dependency-free canvas scatter of the Pareto frontier:
// settling time (x) against maximum joint overshoot (y).

function niceTicks(min, max, count = 5) {
    if (min === max) {
        return [min];
    }
    const span = max - min;
    const step = Math.pow(10, Math.floor(Math.log10(span / count)));
    const multiple = [1, 2, 5, 10].find(m => span / (m * step) <= count) * step;
    const ticks = [];
    for (let v = Math.ceil(min / multiple) * multiple; v <= max + 1e-9; v += multiple) {
        ticks.push(Number(v.toFixed(10)));
    }
    return ticks;
}

function drawPareto(points) {
    const canvas = document.getElementById("paretoCanvas");
    const ctx = canvas.getContext("2d");

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (!points.length) {
        ctx.fillStyle = "#6d6a63";
        ctx.font = "15px Arial";
        ctx.fillText("No Pareto data loaded.", 30, 40);
        return;
    }

    const pad = { left: 70, right: 30, top: 25, bottom: 55 };
    const w = canvas.width - pad.left - pad.right;
    const h = canvas.height - pad.top - pad.bottom;

    const xs = points.map(p => p.settling_time);
    const ys = points.map(p => p.max_overshoot);

    // Pad the ranges so points never sit on the axes.
    const spanX = Math.max(...xs) - Math.min(...xs) || Math.abs(xs[0]) * 0.1 || 1;
    const spanY = Math.max(...ys) - Math.min(...ys) || Math.abs(ys[0]) * 0.1 || 1;
    const minX = Math.min(...xs) - spanX * 0.08;
    const maxX = Math.max(...xs) + spanX * 0.08;
    const minY = Math.max(0, Math.min(...ys) - spanY * 0.08);
    const maxY = Math.max(...ys) + spanY * 0.08;

    const sx = x => pad.left + ((x - minX) / (maxX - minX)) * w;
    const sy = y => pad.top + h - ((y - minY) / (maxY - minY)) * h;

    ctx.font = "12px Arial";
    ctx.lineWidth = 1;

    niceTicks(minX, maxX).forEach(tick => {
        ctx.strokeStyle = "#ece8de";
        ctx.beginPath();
        ctx.moveTo(sx(tick), pad.top);
        ctx.lineTo(sx(tick), pad.top + h);
        ctx.stroke();
        ctx.fillStyle = "#6d6a63";
        ctx.textAlign = "center";
        ctx.fillText(tick.toString(), sx(tick), pad.top + h + 18);
    });

    niceTicks(minY, maxY).forEach(tick => {
        ctx.strokeStyle = "#ece8de";
        ctx.beginPath();
        ctx.moveTo(pad.left, sy(tick));
        ctx.lineTo(pad.left + w, sy(tick));
        ctx.stroke();
        ctx.fillStyle = "#6d6a63";
        ctx.textAlign = "right";
        ctx.fillText(tick.toString(), pad.left - 8, sy(tick) + 4);
    });

    ctx.strokeStyle = "#cfcac0";
    ctx.beginPath();
    ctx.moveTo(pad.left, pad.top);
    ctx.lineTo(pad.left, pad.top + h);
    ctx.lineTo(pad.left + w, pad.top + h);
    ctx.stroke();

    ctx.fillStyle = "#6d6a63";
    ctx.font = "13px Arial";
    ctx.textAlign = "center";
    ctx.fillText("Settling time (s)", pad.left + w / 2, canvas.height - 12);

    ctx.save();
    ctx.translate(18, pad.top + h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("Max overshoot (%)", 0, 0);
    ctx.restore();

    ctx.fillStyle = "#315c4a";
    points.forEach(point => {
        ctx.beginPath();
        ctx.arc(sx(point.settling_time), sy(point.max_overshoot), 5, 0, Math.PI * 2);
        ctx.fill();
    });
}
