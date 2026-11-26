# Two-Link Robot Arm Simulation

A planar two-link robot arm under gravity: rigid-body dynamics, PD and
inverse-dynamics feedforward control, quintic trajectory planning, an
interactive pygame view, and a controller-gain study with Pareto analysis.

## Features

- Full two-link dynamics `M(q)q̈ + C(q,q̇) + G(q) = τ` (forward and inverse)
- Forward and inverse kinematics with reachability check
- Control laws: PD + gravity compensation, PD + inverse-dynamics feedforward
- Quintic (5th-order) joint trajectories, sized from speed and acceleration
  limits, with smooth re-planning mid-move
- Stable fixed-step integration (1 ms sub-steps, frame-time clamp)
- Optional actuator torque limits
- Interactive view: click a point to move the hand, slider panel, ghost
  reference arm and planned path
- Experiments: 2 × 400-run Kp/Kd sweeps, weighted ranking, Pareto frontier,
  step vs trajectory vs feedforward comparison, torque vs move duration

## Run

```bash
uv sync
```

```bash
uv run robot-sim
```

```bash
uv run robot-metrics
```

```bash
uv run pytest
```

Live view keys: click = target, `TAB` = slider panel, `F` = control law,
`L` = torque limits, `G` = gravity, `Q`/`ESC` = quit.

## Layout

```
src/two_link_robot_simulation/
  robot.py           links and robot model
  kinematics.py      forward / inverse kinematics
  dynamics.py        mass matrix, Coriolis, gravity, forward / inverse dynamics
  controller.py      PD, PD + gravity, feedforward, angle wrapping
  trajectory.py      quintic trajectory planning
  integrator.py      sub-stepped time integration, torque limits
  simulation.py      interactive main loop
  pygame_view.py     drawing
  tkinter_controls.py  slider panel
  metrics.py         experiments, metrics, CSVs, plots
tests/               pytest suite
data/processed/controller/   experiment output
```

## Key results (experiment arm 2 m/10 kg + 1 m/5 kg, move 0 → [π/2, π/4])

| Kp=100, Kd=85 | Peak torque | Settling | Tracking RMS |
|---|---|---|---|
| Step + PD/gravity | 391.6 N·m | 4.18 s | 0.584 rad |
| Trajectory + PD/gravity | 282.8 N·m | 4.65 s | 0.193 rad |
| Trajectory + feedforward | 294.2 N·m | 2.19 s | 0.003 rad |

About 221 N·m of each value is the static gravity load of the arm held
horizontal. With feedforward, all 400 gain pairs settle; with PD + gravity,
37 of 400 settle within the 6 s run.
