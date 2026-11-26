"""Quintic joint-space trajectories.

A quintic polynomial is the lowest order that can match position, velocity
and acceleration at both ends. Moves therefore start from the robot's current
state and end at rest with zero acceleration: no step in the reference, so no
torque spike when a new target is set.
"""

import numpy as np

from .controller import wrap_angle

# Joint limits used to size the move duration. Chosen for the default
# 1 m / 5 kg links: a pi rad move takes ~3 s, a pi/2 rad move ~1.7 s.
# Peak torque stays ~140 N m (98 N m of it static gravity), inside the
# optional actuator limits in integrator.DEFAULT_TORQUE_LIMITS.
MAX_JOINT_VELOCITY = 2.0  # rad/s
MAX_JOINT_ACCELERATION = 3.0  # rad/s^2
MIN_DURATION = 0.5  # s

# Peak |velocity| and |acceleration| of a rest-to-rest quintic of
# displacement D over time T: (15/8) D/T and (10/sqrt(3)) D/T^2.
_PEAK_VELOCITY_FACTOR = 15 / 8
_PEAK_ACCELERATION_FACTOR = 10 / np.sqrt(3)


def minimum_duration(
    displacement: np.ndarray,
    max_velocity: float = MAX_JOINT_VELOCITY,
    max_acceleration: float = MAX_JOINT_ACCELERATION,
    initial_velocity: np.ndarray | None = None,
) -> float:
    """Shortest duration that keeps every joint inside the limits.

    All joints share one duration, so they start and finish together.
    """
    distance = np.max(np.abs(displacement))

    if initial_velocity is not None:
        # Braking from the current speed costs extra distance; this keeps
        # re-planned moves from demanding a sudden reversal.
        speed = np.max(np.abs(initial_velocity))
        distance += speed**2 / (2 * max_acceleration)

    velocity_time = _PEAK_VELOCITY_FACTOR * distance / max_velocity
    acceleration_time = np.sqrt(_PEAK_ACCELERATION_FACTOR * distance / max_acceleration)

    return float(max(MIN_DURATION, velocity_time, acceleration_time))


class QuinticTrajectory:
    def __init__(
        self,
        q_start: np.ndarray,
        q_end: np.ndarray,
        t_start: float,
        duration: float,
        v_start: np.ndarray | None = None,
        a_start: np.ndarray | None = None,
    ) -> None:
        if duration <= 0:
            raise ValueError("duration must be positive")

        q0 = np.asarray(q_start, dtype=float)
        qf = np.asarray(q_end, dtype=float)
        v0 = np.zeros(2) if v_start is None else np.asarray(v_start, dtype=float)
        a0 = np.zeros(2) if a_start is None else np.asarray(a_start, dtype=float)

        T = duration
        D = qf - q0

        # Boundary conditions: q, q_dot, q_ddot at start; rest at the end.
        self.coefficients = np.array(
            [
                q0,
                v0,
                a0 / 2,
                (20 * D - 12 * v0 * T - 3 * a0 * T**2) / (2 * T**3),
                (-30 * D + 16 * v0 * T + 3 * a0 * T**2) / (2 * T**4),
                (12 * D - 6 * v0 * T - a0 * T**2) / (2 * T**5),
            ]
        )

        self.q_start = q0
        self.q_end = qf
        self.t_start = t_start
        self.duration = T

    @property
    def t_end(self) -> float:
        return self.t_start + self.duration

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Reference position, velocity and acceleration at time t."""
        s = t - self.t_start

        if s <= 0:
            s = 0.0
        elif s >= self.duration:
            return self.q_end.copy(), np.zeros(2), np.zeros(2)

        c = self.coefficients

        q = c[0] + c[1] * s + c[2] * s**2 + c[3] * s**3 + c[4] * s**4 + c[5] * s**5
        q_dot = (
            c[1] + 2 * c[2] * s + 3 * c[3] * s**2 + 4 * c[4] * s**3 + 5 * c[5] * s**4
        )
        q_ddot = 2 * c[2] + 6 * c[3] * s + 12 * c[4] * s**2 + 20 * c[5] * s**3

        return q, q_dot, q_ddot

    def is_finished(self, t: float) -> bool:
        return t >= self.t_end


def plan(
    q_start: np.ndarray,
    q_target: np.ndarray,
    t_start: float,
    v_start: np.ndarray | None = None,
    a_start: np.ndarray | None = None,
    duration: float | None = None,
    max_velocity: float = MAX_JOINT_VELOCITY,
    max_acceleration: float = MAX_JOINT_ACCELERATION,
) -> QuinticTrajectory:
    """Plan a move from the current state to q_target along the shortest way.

    The end point is expressed next to q_start (q_start + wrapped difference),
    so the reference never jumps by 2*pi.
    """
    q_start = np.asarray(q_start, dtype=float)
    displacement = wrap_angle(np.asarray(q_target, dtype=float) - q_start)

    if duration is None:
        duration = minimum_duration(
            displacement, max_velocity, max_acceleration, v_start
        )

    return QuinticTrajectory(
        q_start,
        q_start + displacement,
        t_start,
        duration,
        v_start,
        a_start,
    )
