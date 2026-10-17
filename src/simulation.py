import numpy as np
import pygame

import dynamics
from controller import PDController
from kinematics import ForwardKinematics, InverseKinematics
from robot import Link, Robot
from tkinter_controls import ControlPanel

HEIGHT = 900
WIDTH = 1600
FRAMES_PER_SECOND = 120
SCALE = 150

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PURPLE = (139, 92, 246)
GREY = (128, 128, 128)

BASE = (WIDTH / 2, HEIGHT / 2)
KP = 30
KD = 30

screen: pygame.Surface
clock: pygame.time.Clock
panel: ControlPanel
font: pygame.font.Font


def event_handler(
    robot: Robot,
    mode: int,
    target: np.ndarray | None,
    q_d: np.ndarray,
    reachable: bool,
    running: bool,
):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False

            elif event.key == pygame.K_g:
                dynamics.g = 0 if dynamics.g != 0 else 9.81

            elif event.key == pygame.K_TAB:
                mode = 2 if mode == 1 else 1

                if mode == 2:
                    panel.show()
                else:
                    panel.hide()

        elif event.type == pygame.MOUSEBUTTONDOWN and mode == 1:
            mouse_x, mouse_y = event.pos

            x = (mouse_x - BASE[0]) / SCALE
            y = (BASE[1] - mouse_y) / SCALE

            target = np.array([x, y])
            new_q_d = InverseKinematics(robot, target)

            if new_q_d is not None:
                q_d = new_q_d
                reachable = True
            else:
                reachable = False

    return mode, target, q_d, reachable, running


def update(
    robot: Robot,
    controller: PDController,
    q: np.ndarray,
    q_dot: np.ndarray,
    q_d: np.ndarray,
    q_d_dot: np.ndarray,
    dt: float,
):
    pd_tau = controller.torque(q, q_dot, q_d, q_d_dot)
    gravity_tau = dynamics.Gravity(robot, q)
    tau = pd_tau + gravity_tau
    q_ddot = dynamics.ForwardDynamics(robot, q, q_dot, tau)

    q_dot += q_ddot * dt
    q += q_dot * dt

    return q, q_dot, tau


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


def main():
    global screen, clock, panel, font

    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 36)

    panel = ControlPanel()
    panel.hide()

    link1 = Link(2, 0, 10)
    link2 = Link()
    robot = Robot([link1, link2])
    controller = PDController(KP, KD)

    q = np.array([0.0, 0.0])
    q_dot = np.array([0.0, 0.0])

    q_d = np.zeros(2)
    q_d_dot = np.array([0.0, 0.0])
    target = None
    reachable = False
    mode = 1
    running = True

    while running:
        panel.root.update()

        mode, target, q_d, reachable, running = event_handler(
            robot,
            mode,
            target,
            q_d,
            reachable,
            running,
        )

        if mode == 2:
            q_d = np.array(
                [
                    panel.theta1.get(),
                    panel.theta2.get(),
                ]
            )

            controller.kp = int(panel.kp.get())
            controller.kd = int(panel.kd.get())

        dt = clock.tick(FRAMES_PER_SECOND) / 1000

        q, q_dot, tau = update(
            robot,
            controller,
            q,
            q_dot,
            q_d,
            q_d_dot,
            dt,
        )

        render(
            robot,
            q,
            q_d,
            q_dot,
            tau,
            target,
            reachable,
            mode,
        )

    pygame.quit()
    panel.close()


if __name__ == "__main__":
    main()
