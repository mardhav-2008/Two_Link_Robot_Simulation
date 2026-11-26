// Small browser-side visualisation of the two-link arm.
// The actual robot model lives in the Python project.

const canvas = document.getElementById("robotCanvas");
const ctx = canvas.getContext("2d");

const link1 = 125;
const link2 = 105;

function drawRobot(theta1, theta2) {
    const width = canvas.width;
    const height = canvas.height;

    ctx.clearRect(0, 0, width, height);

    // simple background grid
    ctx.strokeStyle = "#dedbd2";
    ctx.lineWidth = 1;

    for (let x = 25; x < width; x += 25) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }

    for (let y = 25; y < height; y += 25) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    const baseX = width / 2;
    const baseY = height / 2 + 80;

    const a1 = theta1 * Math.PI / 180;
    const a2 = (theta1 + theta2) * Math.PI / 180;

    const jointX = baseX + link1 * Math.cos(a1);
    const jointY = baseY - link1 * Math.sin(a1);

    const endX = jointX + link2 * Math.cos(a2);
    const endY = jointY - link2 * Math.sin(a2);

    // first link
    drawLink(baseX, baseY, jointX, jointY);

    // second link
    drawLink(jointX, jointY, endX, endY);

    // joints
    drawJoint(baseX, baseY, 9);
    drawJoint(jointX, jointY, 8);
    drawJoint(endX, endY, 7);

    document.getElementById("endX").textContent =
        ((endX - baseX) / 100).toFixed(2);

    document.getElementById("endY").textContent =
        ((baseY - endY) / 100).toFixed(2);
}

function drawLink(x1, y1, x2, y2) {
    ctx.strokeStyle = "#315c4a";
    ctx.lineWidth = 16;
    ctx.lineCap = "round";

    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();

    ctx.strokeStyle = "#263c33";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
}

function drawJoint(x, y, radius) {
    ctx.fillStyle = "#fffdf8";
    ctx.strokeStyle = "#202020";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
}

function updateRobot() {
    const theta1 = Number(document.getElementById("theta1").value);
    const theta2 = Number(document.getElementById("theta2").value);

    document.getElementById("theta1Value").textContent = `${theta1}°`;
    document.getElementById("theta2Value").textContent = `${theta2}°`;

    drawRobot(theta1, theta2);
}

document.getElementById("theta1").addEventListener("input", updateRobot);
document.getElementById("theta2").addEventListener("input", updateRobot);

document.getElementById("resetBtn").addEventListener("click", () => {
    document.getElementById("theta1").value = 35;
    document.getElementById("theta2").value = 55;
    updateRobot();
});

updateRobot();
