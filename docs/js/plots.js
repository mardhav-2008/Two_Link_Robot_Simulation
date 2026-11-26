// Basic canvas plot.
// Keeping this dependency-free makes the GitHub Pages site easy to run.

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

    const pad = 55;
    const w = canvas.width - pad * 2;
    const h = canvas.height - pad * 2;

    const xs = points.map(p => p.x);
    const ys = points.map(p => p.y);

    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);

    function sx(x) {
        return pad + ((x - minX) / (maxX - minX || 1)) * w;
    }

    function sy(y) {
        return canvas.height - pad -
            ((y - minY) / (maxY - minY || 1)) * h;
    }

    ctx.strokeStyle = "#cfcac0";
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.moveTo(pad, pad);
    ctx.lineTo(pad, canvas.height - pad);
    ctx.lineTo(canvas.width - pad, canvas.height - pad);
    ctx.stroke();

    ctx.fillStyle = "#6d6a63";
    ctx.font = "13px Arial";
    ctx.fillText("Settling Time", canvas.width / 2 - 35, canvas.height - 15);

    ctx.save();
    ctx.translate(16, canvas.height / 2 + 35);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("Overshoot", 0, 0);
    ctx.restore();

    points.forEach(point => {
        ctx.fillStyle = "#315c4a";
        ctx.beginPath();
        ctx.arc(sx(point.x), sy(point.y), 5, 0, Math.PI * 2);
        ctx.fill();
    });
}
