2-Link Robot Simulation

A Python-based simulation of a 2-Degree-of-Freedom (2-DOF) planar robotic manipulator, implementing forward and inverse kinematics, Lagrangian dynamics, and feedback control.

The project explores the mathematical modelling, simulation, and control of a two-link robotic arm, with an emphasis on understanding its physical behaviour and evaluating controller performance through numerical experiments.

Overview

Two-link robotic manipulators are fundamental systems in robotics, providing a relatively simple yet powerful platform for studying robot kinematics, dynamics, and control.

This project implements a mathematical model of a planar robotic arm with two revolute joints. It uses forward and inverse kinematics to determine the end-effector position and joint configurations, while its dynamic model describes the relationship between joint torques, positions, velocities, and accelerations.

A feedback controller is used to regulate joint motion and achieve desired configurations. Controller experiments are also conducted to investigate the effects of different control parameters on system performance.

Features

- Forward Kinematics: Calculates the end-effector position from joint angles.
- Inverse Kinematics: Determines the required joint angles for a desired end-effector position.
- Reachability Verification: Checks whether a target position lies within the manipulator's reachable workspace.
- Dynamic Modelling: Implements the equations of motion using Lagrangian mechanics.
- Numerical Simulation: Simulates the manipulator's joint behaviour over time.
- PD Control: Implements proportional-derivative feedback control.
- Controller Experiments: Evaluates different controller parameter combinations.
- Performance Metrics: Analyses controller performance using settling time, overshoot, torque requirements, and an overall performance score.

Mathematical Foundation

1. Forward Kinematics

For a planar two-link manipulator, the end-effector position is calculated using:

[
x=L_1\cos(q_1)+L_2\cos(q_1+q_2)
]

[
y=L_1\sin(q_1)+L_2\sin(q_1+q_2)
]

Where:

- (L_1,L_2): Link lengths
- (q_1,q_2): Joint angles
- (x,y): End-effector coordinates

2. Inverse Kinematics

The inverse kinematics problem determines the joint angles required to reach a specified Cartesian position.

The second joint angle is calculated using:

[
\cos(q_2)=
\frac{x^2+y^2-L_1^2-L_2^2}
{2L_1L_2}
]

The first joint angle is obtained using:

[
q_1=\operatorname{atan2}(y,x)-
\operatorname{atan2}
(L_2\sin(q_2),L_1+L_2\cos(q_2))
]

Unreachable target positions are identified before calculating a solution.

3. Lagrangian Dynamics

The manipulator's equations of motion are derived using Lagrangian mechanics.

The general dynamic equation is:

[
M(q)\ddot q+C(q,\dot q)+G(q)=\tau
]

Where:

- (M(q)): Joint-space inertia matrix
- (C(q,\dot q)): Coriolis and centrifugal terms
- (G(q)): Gravitational torque vector
- (\tau): Applied joint torque
- (q,\dot q,\ddot q): Joint positions, velocities, and accelerations

Joint accelerations are obtained by solving:

[
\ddot q=M(q)^{-1}
[\tau-C(q,\dot q)-G(q)]
]

The implementation uses numerical linear-system solving rather than explicitly calculating the matrix inverse.

4. PD Controller

The proportional-derivative controller generates joint torques based on position and velocity errors.

[
\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)
]

Where:

- (K_p): Proportional gain
- (K_d): Derivative gain
- (q_d): Desired joint position
- (q): Actual joint position

The controller is evaluated using different gain combinations to investigate stability, tracking performance, and torque requirements.

Controller Evaluation

Controller experiments evaluate the effect of different proportional and derivative gains.

The evaluation considers:

Metric| Description
Settling Time| Time taken for the system to settle near its target
Overshoot| Amount by which the response exceeds the target
Torque| Control effort required by the controller
Performance Score| Combined metric used to compare controller configurations

An example result from the controller experiments:

Parameter| Value
Proportional Gain ((K_p))| 100
Derivative Gain ((K_d))| 85
Performance Score| 0.4690
Settling Time| 4.171 s
Overshoot| 8.07%
Torque| 391.61

These values represent an experimental controller configuration, not a universal optimum.

Project Structure

Two_Link_Robot_Simulation/
│
├── src/
│   └── two_link_robot_simulation/
│       ├── __init__.py
│       ├── kinematics.py
│       ├── dynamics.py
│       ├── controller.py
│       ├── robot.py
│       ├── simulation.py
│       ├── pygame_view.py
│       └── tkinter_controls.py
│
├── data/
│   ├── raw/
│   ├── interim/
│   └── processed/
│       └── controller/
│
├── tests/
│
├── pyproject.toml
├── uv.lock
└── README.md

Technologies Used

- Python 3.14+ – Core programming language
- NumPy – Numerical computations and matrix operations
- Plotly – Data visualization
- Pygame-CE – Interactive simulation visualization
- Tkinter – Simulation controls
- uv – Python dependency and project management
- pytest – Testing

Installation

Prerequisites

- Python 3.14 or later
- "uv" (https://docs.astral.sh/uv/)

1. Clone the repository

git clone <REPOSITORY_URL>
cd Two_Link_Robot_Simulation

2. Install dependencies

uv sync

This creates the virtual environment and installs the project dependencies.

3. Run the simulation

uv run python -m two_link_robot_simulation.simulation

4. Run controller experiments

uv run python -m two_link_robot_simulation.metrics

Experimental results are saved under:

data/processed/controller/

Testing

Run the test suite using:

uv run pytest

Objectives

The primary objectives of this project are:

1. Develop a mathematical model of a planar two-link robotic manipulator.
2. Implement forward and inverse kinematics.
3. Derive and implement equations of motion using Lagrangian mechanics.
4. Simulate the dynamic behaviour of the manipulator.
5. Implement feedback control for joint-position regulation.
6. Evaluate controller performance under different parameter configurations.
7. Develop an interactive environment for observing and analysing robot behaviour.

Future Improvements

Potential extensions to the project include:

- Trajectory generation and tracking.
- Computed-torque control.
- Jacobian-based velocity kinematics.
- Comparative analysis of different control strategies.
- Workspace and singularity analysis.
- More extensive controller optimization.
- Validation against an independent physics-based simulation.

Contributors

- Mardhav Rayapati
- Jinadev RP
- Hari Ramachandran
- Pradeep S

---

License

To be determined.