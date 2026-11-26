# Two-Link Robot Simulation

A Python simulation of a 2-degree-of-freedom (2-DOF) planar robotic arm moving
under gravity. It implements forward and inverse kinematics, Lagrangian
dynamics, PD and inverse-dynamics feedforward control, and quintic trajectory
planning. It also runs controller-gain experiments with Pareto analysis.

The project explores the mathematical modelling, simulation and control of a
two-link arm. The focus is on its physical behaviour and on measuring
controller performance through numerical experiments.

## Overview

Two-link manipulators are a fundamental system in robotics: simple enough to
model completely, but rich enough to show coupling between joints, gravity
loading and the trade-offs of feedback control.

This project models a planar arm with two revolute joints:

- **Kinematics** maps joint angles to the hand position and back.
- **Dynamics** relates joint torques to positions, velocities and accelerations.
- **Trajectory planning** turns a target into a smooth, time-parameterised
  motion.
- **Controllers** compute the joint torques that make the arm follow that
  motion.
- **Experiments** sweep controller gains and compare control strategies on
  settling time, overshoot, torque and tracking accuracy.

## Features

- **Forward kinematics:** end-effector position from joint angles.
- **Inverse kinematics:** joint angles for a target position, with a
  workspace reachability check.
- **Lagrangian dynamics:** `M(q)q̈ + C(q,q̇) + G(q) = τ`, with forward and
  inverse dynamics.
- **Control laws:**
  - PD with gravity compensation.
  - PD with inverse-dynamics feedforward.
  - Joint errors wrapped to the shortest way round.
- **Quintic trajectories:**
  - Smooth moves that end at rest with zero acceleration.
  - Duration sized from joint speed and acceleration limits.
  - Continuous re-planning when the target changes mid-move.
- **Stable integration:**
  - Fixed 1 ms semi-implicit Euler sub-steps.
  - Frame-time clamp.
- **Actuator limits:** optional joint torque saturation.
- **Interactive simulation:**
  - Click to set a target.
  - Slider panel for angles, gains and speed limit.
  - Ghost reference arm and planned hand path.
  - Live torque and tracking readouts.
- **Controller experiments:**
  - Two 400-run Kp/Kd sweeps, one per control law.
  - Weighted ranking and Pareto frontier.
  - Step vs trajectory vs feedforward comparison.
  - Peak torque vs move duration.
- **Project website** (`docs/`): interactive arm and exported experiment
  results for GitHub Pages.

## Mathematical foundation

### 1. Forward kinematics

$$
x = L_1\cos q_1 + L_2\cos(q_1+q_2), \qquad
y = L_1\sin q_1 + L_2\sin(q_1+q_2)
$$

where $L_1, L_2$ are the link lengths, $q_1$ is the angle of link 1 from the
horizontal and $q_2$ is the angle of link 2 relative to link 1.

### 2. Inverse kinematics

A target is reachable only if $|L_1-L_2| \le \sqrt{x^2+y^2} \le L_1+L_2$. Then

$$
\cos q_2 = \frac{x^2+y^2-L_1^2-L_2^2}{2L_1L_2}, \qquad
q_1 = \operatorname{atan2}(y,x) - \operatorname{atan2}\!\left(L_2\sin q_2,\; L_1+L_2\cos q_2\right)
$$

The elbow-down solution ($q_2 \ge 0$) is used.

### 3. Lagrangian dynamics

$$
M(q)\,\ddot q + C(q,\dot q) + G(q) = \tau
$$

- $M(q)$: joint-space inertia (mass) matrix
- $C(q,\dot q)$: Coriolis and centrifugal terms
- $G(q)$: gravity torques
- $\tau$: applied joint torques

Accelerations come from solving $M\ddot q = \tau - C - G$ as a linear system,
not from an explicit matrix inverse.

### 4. Control

PD with gravity compensation:

$$
\tau = K_p\,\mathrm{wrap}(q_{ref}-q) + K_d(\dot q_{ref}-\dot q) + G(q)
$$

PD with inverse-dynamics feedforward:

$$
\tau = K_p\,\mathrm{wrap}(q_{ref}-q) + K_d(\dot q_{ref}-\dot q) + M(q)\ddot q_{ref} + C(q,\dot q) + G(q)
$$

$\mathrm{wrap}$ maps an angle to $[-\pi,\pi)$, so a joint never turns the
long way round.

### 5. Quintic trajectories

Each joint follows $q(s)=\sum_{k=0}^{5} a_k s^k$, which matches position,
velocity and acceleration at both ends. The duration $T$ is the smallest value
that keeps the peaks $\tfrac{15}{8}D/T$ (velocity) and $\tfrac{10}{\sqrt3}D/T^2$
(acceleration) inside the joint limits. Following a smooth reference instead
of a step target removes the $K_p \times$ step torque spike.

## Results

Experiment arm 2 m / 10 kg + 1 m / 5 kg, moving from horizontal to
$[\pi/2, \pi/4]$, 6 s runs at 240 Hz:

| Kp = 100, Kd = 85 | Peak torque | Settling time | Tracking RMS |
|---|---|---|---|
| Step target + PD/gravity | 391.6 N·m | 4.18 s | 0.584 rad |
| Trajectory + PD/gravity | 282.8 N·m | 4.65 s | 0.193 rad |
| Trajectory + feedforward | 294.2 N·m | 2.19 s | 0.003 rad |

- The trajectory cuts peak torque by 28% and removes the jolt at the start of
  each move.
- Feedforward settles about twice as fast and follows the plan almost exactly.
  All 400 gain pairs settle in about 2.19 s.
- With PD + gravity, 37 of 400 gain pairs settle within 6 s, all with Kp and
  Kd at or above about 65. The best is Kp = 100, Kd = 100: settles in 4.85 s
  with 16% overshoot.
- About 221 N·m of every peak is the static gravity load of the horizontal
  arm, the minimum any controller needs.

These are results for one test move on one arm, not a universal optimum.

## Project structure

```
Two_Link_Robot_Simulation/
├── src/two_link_robot_simulation/
│   ├── robot.py              links and robot model
│   ├── kinematics.py         forward / inverse kinematics
│   ├── dynamics.py           mass matrix, Coriolis, gravity, forward / inverse dynamics
│   ├── controller.py         PD, PD + gravity, feedforward, angle wrapping
│   ├── trajectory.py         quintic trajectory planning
│   ├── integrator.py         sub-stepped integration, torque limits
│   ├── simulation.py         interactive main loop
│   ├── pygame_view.py        drawing
│   ├── tkinter_controls.py   slider panel
│   └── metrics.py            experiments, metrics, CSVs, plots
├── tests/                    pytest suite
├── data/processed/controller/
│   ├── pd_gravity/           sweep results for PD + gravity
│   ├── feedforward/          sweep results for feedforward
│   └── *.csv, *.png          comparison, tracking and duration studies
├── scripts/export_web_results.py   CSV results to JSON for the website
├── docs/                     GitHub Pages site
├── pyproject.toml
└── uv.lock
```

## Technologies

- Python 3.14+
- NumPy: numerical computation and linear algebra
- Matplotlib: experiment plots
- Pygame-CE: interactive simulation view
- Tkinter: slider control panel
- uv: dependency and project management
- pytest: testing
- HTML, CSS and JavaScript: project website

## Installation and usage

Prerequisites: Python 3.14 or later and [uv](https://docs.astral.sh/uv/).

```bash
git clone https://github.com/mardhav-2008/Two_Link_Robot_Simulation.git
cd Two_Link_Robot_Simulation
uv sync
```

Run the interactive simulation:

```bash
uv run robot-sim
```

| Input | Action |
|---|---|
| Click | Move the hand to the clicked point |
| `TAB` | Switch between click mode and the slider panel |
| `F` | Switch control law (PD + gravity or feedforward) |
| `L` | Toggle actuator torque limits |
| `G` | Toggle gravity |
| `Q` / `ESC` | Quit |

Run the controller experiments. They write to `data/processed/controller/`,
take about 1.5 minutes, and can run from any folder:

```bash
uv run robot-metrics
```

Export the results for the website:

```bash
uv run python scripts/export_web_results.py
```

Run the tests:

```bash
uv run pytest
```

## Objectives

1. Develop a mathematical model of a planar two-link robotic manipulator.
2. Implement forward and inverse kinematics.
3. Derive and implement the equations of motion using Lagrangian mechanics.
4. Simulate the dynamic behaviour of the manipulator.
5. Implement feedback and feedforward control for trajectory tracking.
6. Evaluate controller performance under different parameter configurations.
7. Provide an interactive environment for observing and analysing robot
   behaviour.

## Future improvements

- Computed-torque control (feedback linearisation).
- Model uncertainty, friction and sensor noise.
- Jacobian-based velocity kinematics and task-space trajectories.
- Workspace and singularity analysis.
- Joint limits and elbow-up inverse kinematics.
- Validation against an independent physics engine.

## Contributors

- Mardhav Rayapati
- Jinadev RP
- Hari Ramachandran
- Pradeep S

## License

To be determined.
