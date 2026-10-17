import numpy as np
import pygame

from . import dynamics
from .controller import PDController
from .kinematics import InverseKinematics
from .pygame_view import BASE, SCALE, initialize, render
from .robot import Link, Robot
from .tkinter_controls import ControlPanel

FRAMES_PER_SECOND = 60

KP = 30
KD = 30


def event_handler(
    robot: Robot,
    panel: ControlPanel,
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


def simulation():

    pygame.init()
    initialize()

    clock = pygame.time.Clock()

    panel = ControlPanel()
    panel.hide()

    link1 = Link()
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

    while running and panel.running:
        panel.root.update()

        mode, target, q_d, reachable, running = event_handler(
            robot,
            panel,
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
    simulation()
