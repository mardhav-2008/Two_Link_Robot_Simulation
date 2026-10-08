import numpy as np

PI = np.pi


class PDController:
    def __init__(self, kp, kd) -> None:
        self.kp = kp
        self.kd = kd

    def torque(
        self,
        q: np.ndarray,
        q_dot: np.ndarray,
        q_d: np.ndarray,
        q_d_dot: np.ndarray,
    ) -> np.ndarray:

        return self.kp * (q_d - q) + self.kd * (q_d_dot - q_dot)
