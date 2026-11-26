import numpy as np

from . import dynamics
from .controller import PD_GRAVITY, PDController, control_torque
from .robot import Robot
from .trajectory import QuinticTrajectory

# Semi-implicit Euler is only stable while Kd * dt * max_eig(M^-1) < 2.
# For the default 1 m / 5 kg links max_eig(M^-1) is about 3, so a single
# 1/60 s step diverges for Kd > ~40. Sub-stepping at 1 ms keeps the whole
# Kp/Kd slider range (0-100) stable.
PHYSICS_DT = 1 / 1000

# A stalled frame (window drag, slow first frame) must not feed a huge dt
# into the physics.
MAX_FRAME_DT = 0.05

# Optional actuator limits (N m) per joint. None = ideal motors.
# Sized for the default 1 m / 5 kg links: ~1.5x the worst static gravity
# load (98 N m and 25 N m with the arm horizontal).
DEFAULT_TORQUE_LIMITS = np.array([150.0, 40.0])


def step(
    robot: Robot,
    controller: PDController,
    q: np.ndarray,
    q_dot: np.ndarray,
    q_d: np.ndarray,
    q_d_dot: np.ndarray,
    dt: float,
    physics_dt: float = PHYSICS_DT,
    *,
    trajectory: QuinticTrajectory | None = None,
    t: float = 0.0,
    mode: str = PD_GRAVITY,
    torque_limits: np.ndarray | None = None,
):
    """Advance the robot by dt seconds using fixed physics sub-steps.

    With a trajectory, the reference is sampled at every sub-step from
    time t; otherwise q_d / q_d_dot are held constant (step reference).

    Returns the new q, q_dot and the torque applied in the last sub-step.
    """
    dt = min(dt, MAX_FRAME_DT)
    substeps = max(1, int(np.ceil(dt / physics_dt - 1e-9)))
    h = dt / substeps

    q = q.copy()
    q_dot = q_dot.copy()
    tau = np.zeros(2)

    q_ref, q_dot_ref, q_ddot_ref = q_d, q_d_dot, np.zeros(2)

    for i in range(substeps):
        if trajectory is not None:
            q_ref, q_dot_ref, q_ddot_ref = trajectory.sample(t + i * h)

        tau = control_torque(
            robot, controller, q, q_dot, q_ref, q_dot_ref, q_ddot_ref, mode
        )

        if torque_limits is not None:
            tau = np.clip(tau, -torque_limits, torque_limits)

        q_ddot = dynamics.ForwardDynamics(robot, q, q_dot, tau)

        q_dot += q_ddot * h
        q += q_dot * h

    return q, q_dot, tau
