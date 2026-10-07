import numpy as np
import plotly.graph_objects as go

PI = np.pi
BASE = (0, 0)


def ForwardKinematics(
    L1: float, L2: float, T1: float, T2: float
) -> tuple[tuple, tuple]:
    x_1 = L1 * np.cos(T1)
    y_1 = L1 * np.sin(T1)

    x_end = L1 * np.cos(T1) + L2 * np.cos(T1 + T2)
    y_end = L1 * np.sin(T1) + L2 * np.sin(T1 + T2)

    return ((x_1, y_1), (x_end, y_end))


if __name__ == "__main__":
    FK = ForwardKinematics

    fk_1 = FK(1, 1, PI / 2, -PI / 4)

    J2 = fk_1[0]
    END = fk_1[1]

    x = [BASE[0], J2[0], END[0]]
    y = [BASE[1], J2[1], END[1]]

    fig = go.Figure()

    fig.add_trace(go.Scatter(x=x, y=y, mode="lines+markers"))

    fig.show()
