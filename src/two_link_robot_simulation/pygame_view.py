import numpy as np
import pygame

from . import dynamics
from .kinematics import ForwardKinematics
from .robot import Robot

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

    if reachable:
        difference_text = font.render(
            f"qd - q = ({q_d[0] - q[0]:.2f}, {q_d[1] - q[1]:.2f})",
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

    pygame.display.flip()
