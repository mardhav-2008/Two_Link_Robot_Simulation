import numpy as np
import pygame

from . import dynamics
from .controller import wrap_angle
from .kinematics import ForwardKinematics
from .robot import Robot
from .trajectory import QuinticTrajectory

WIDTH = 1600
HEIGHT = 900
SCALE = 150

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PURPLE = (139, 92, 246)
GREY = (128, 128, 128)
DARK_GREY = (70, 70, 70)
CYAN = (34, 211, 238)

# Number of samples used to draw the planned end-effector path.
PATH_SAMPLES = 60

BASE = (WIDTH / 2, HEIGHT / 2)

screen: pygame.Surface
font: pygame.font.Font


def initialize():
    global screen, font

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    font = pygame.font.Font(None, 36)


def render(
    robot: Robot,
    q: np.ndarray,
    q_d: np.ndarray,
    q_dot: np.ndarray,
    tau: np.ndarray,
    target: np.ndarray | None,
    reachable: bool,
    mode: int,
    q_ref: np.ndarray | None = None,
    trajectory: QuinticTrajectory | None = None,
    t: float = 0.0,
    control_mode: str = "pd_gravity",
    torque_limited: bool = False,
):
    fk = ForwardKinematics(robot, q)

    screen.fill(BLACK)

    link1, link2 = robot.links

    J2 = (
        BASE[0] + fk[0][0] * SCALE,
        BASE[1] - fk[0][1] * SCALE,
    )

    END = (
        BASE[0] + fk[1][0] * SCALE,
        BASE[1] - fk[1][1] * SCALE,
    )

    mode_text = font.render(f"MODE = {mode}", True, WHITE)
    gravity_text = font.render(f"g = {dynamics.g}", True, WHITE)

    q_text = font.render(
        f"theta1 = {q[0]:.2f}, theta2 = {q[1]:.2f}",
        True,
        WHITE,
    )

    q_dot_text = font.render(
        f"theta1_dot = {q_dot[0]:.2f}, theta2_dot = {q_dot[1]:.2f}",
        True,
        WHITE,
    )

    tau_text = font.render(
        f"tau1 = {tau[0]:.2f}, tau2 = {tau[1]:.2f}",
        True,
        WHITE,
    )

    if reachable or mode == 2:
        goal_error = wrap_angle(q_d - q)
        difference_text = font.render(
            f"goal - q = ({goal_error[0]:.2f}, {goal_error[1]:.2f})",
            True,
            WHITE,
        )
    else:
        difference_text = font.render(
            "Point is not reachable",
            True,
            WHITE,
        )

    pygame.draw.circle(
        screen,
        PURPLE,
        BASE,
        (link1.length + link2.length) * SCALE,
        2,
    )

    pygame.draw.circle(
        screen,
        GREY,
        BASE,
        abs(link1.length - link2.length) * SCALE,
        2,
    )

    if target is not None and mode == 1:
        target_screen = (
            BASE[0] + target[0] * SCALE,
            BASE[1] - target[1] * SCALE,
        )

        target_color = RED if reachable else GREY
        pygame.draw.circle(screen, target_color, target_screen, 8)

    if trajectory is not None:
        draw_planned_path(robot, trajectory, t)

    if q_ref is not None:
        draw_reference_arm(robot, q_ref)

    pygame.draw.aaline(screen, WHITE, BASE, J2)
    pygame.draw.aaline(screen, WHITE, J2, END)

    pygame.draw.circle(screen, RED, BASE, 8)
    pygame.draw.circle(screen, GREEN, J2, 8)
    pygame.draw.circle(screen, BLUE, END, 8)

    screen.blit(mode_text, (20, 0))
    screen.blit(gravity_text, (20, 30))
    screen.blit(q_text, (20, 60))
    screen.blit(q_dot_text, (20, 90))
    screen.blit(tau_text, (20, 120))
    screen.blit(difference_text, (20, 150))

    control_text = font.render(
        f"control = {control_mode} [F]   torque limit = "
        f"{'on' if torque_limited else 'off'} [L]",
        True,
        WHITE,
    )
    screen.blit(control_text, (20, 180))

    if q_ref is not None:
        tracking_error = wrap_angle(q_ref - q)
        tracking_text = font.render(
            f"tracking error = ({tracking_error[0]:.3f}, {tracking_error[1]:.3f})",
            True,
            CYAN,
        )
        screen.blit(tracking_text, (20, 210))

    if trajectory is not None:
        remaining = max(0.0, trajectory.t_end - t)
        move_text = font.render(
            f"move time = {trajectory.duration:.2f} s, remaining = {remaining:.2f} s",
            True,
            WHITE,
        )
        screen.blit(move_text, (20, 240))

    pygame.display.flip()


def to_screen(point) -> tuple[float, float]:
    return (BASE[0] + point[0] * SCALE, BASE[1] - point[1] * SCALE)


def draw_reference_arm(robot: Robot, q_ref: np.ndarray):
    """Ghost arm at the planned (reference) pose."""
    elbow, end = ForwardKinematics(robot, q_ref)

    pygame.draw.line(screen, DARK_GREY, BASE, to_screen(elbow), 3)
    pygame.draw.line(screen, DARK_GREY, to_screen(elbow), to_screen(end), 3)


def draw_planned_path(robot: Robot, trajectory: QuinticTrajectory, t: float):
    """Remaining end-effector path of the current move."""
    if trajectory.is_finished(t):
        return

    times = np.linspace(t, trajectory.t_end, PATH_SAMPLES)
    points = [to_screen(ForwardKinematics(robot, trajectory.sample(s)[0])[1]) for s in times]

    # Every other segment: a dashed line.
    for a, b in zip(points[0:-1:2], points[1::2]):
        pygame.draw.aaline(screen, CYAN, a, b)
