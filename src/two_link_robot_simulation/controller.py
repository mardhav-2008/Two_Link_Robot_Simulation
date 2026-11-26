import numpy as np

from . import dynamics
from .robot import Robot

# Control laws available to the simulation and the experiments.
#   "pd_gravity":  tau = PD + G(q)
#       Gravity compensation only; the PD terms supply all motion torque.
#   "feedforward": tau = PD + M(q) q_ddot_ref + C(q, q_dot) + G(q)
#       Inverse-dynamics feedforward: the model supplies the torque the
#       trajectory needs and the PD terms only correct tracking error.
PD_GRAVITY = "pd_gravity"
FEEDFORWARD = "feedforward"
CONTROL_MODES = (PD_GRAVITY, FEEDFORWARD)


class PDController:
    def __init__(self, kp: float, kd: float) -> None:
        self.kp = kp
        self.kd = kd

    def torque(
        self,
        q: np.ndarray,
        q_dot: np.ndarray,
        q_d: np.ndarray,
        q_d_dot: np.ndarray,
    ) -> np.ndarray:
        # Revolute joints: take the shortest way round, so an IK solution
        # of -3.9 rad is not chased the long way from +3.0 rad.
        error = wrap_angle(q_d - q)

        return self.kp * error + self.kd * (q_d_dot - q_dot)


def control_torque(
    robot: Robot,
    controller: PDController,
    q: np.ndarray,
    q_dot: np.ndarray,
    q_ref: np.ndarray,
    q_dot_ref: np.ndarray,
    q_ddot_ref: np.ndarray,
    mode: str = PD_GRAVITY,
) -> np.ndarray:
    pd_tau = controller.torque(q, q_dot, q_ref, q_dot_ref)

    if mode == PD_GRAVITY:
        return pd_tau + dynamics.Gravity(robot, q)

    if mode == FEEDFORWARD:
        return pd_tau + dynamics.InverseDynamics(robot, q, q_dot, q_ddot_ref)

    raise ValueError(f"unknown control mode: {mode!r}")


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    """Wrap angles to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi
