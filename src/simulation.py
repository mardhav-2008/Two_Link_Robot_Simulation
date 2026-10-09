import numpy as np
import pygame

import dynamics
from controller import PDController
from kinematics import ForwardKinematics, InverseKinematics
from robot import Link, Robot
from tkinter_controls import ControlPanel

HEIGHT = 720
WIDTH = 1080
FRAMES_PER_SECOND = 120
SCALE = 150

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

PI = np.pi
BASE = (WIDTH / 2, HEIGHT / 2)
KP = 30
KD = 30


def main():

    link1 = Link()
    link2 = Link()
    robot = Robot([link1, link2])
    controller = PDController(KP, KD)

    q = np.array([0.0, 0.0])
    q_dot = np.array([0.0, 0.0])

    q_d = np.zeros(2)
    q_d_dot = np.array([0.0, 0.0])
    target = None

    pygame.init()
    mode = 1
    panel = ControlPanel()
    panel.hide()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 36)

    running = True

    while running:
        panel.root.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_g:
                    dynamics.g = 0 if dynamics.g != 0 else 9.81

            if event.type == pygame.KEYDOWN and event.key == pygame.K_TAB:
                mode = 2 if mode == 1 else 1

                if mode == 2:
                    panel.show()

                if mode == 1:
                    panel.hide()

            elif event.type == pygame.MOUSEBUTTONDOWN and mode == 1:
                mouse_x, mouse_y = event.pos

                x = (mouse_x - BASE[0]) / SCALE
                y = (BASE[1] - mouse_y) / SCALE

                target = np.array([x, y])

                q_d = InverseKinematics(robot, target)
        if mode == 2:
            theta1 = panel.theta1.get()
            theta2 = panel.theta2.get()
            kp = panel.kp.get()
            kd = panel.kd.get()

            controller.kp = int(kp)
            controller.kd = int(kd)

            q_d = np.array([theta1, theta2])

        dt = clock.tick(FRAMES_PER_SECOND) / 1000
        screen.fill(BLACK)

        pd_tau = controller.torque(q, q_dot, q_d, q_d_dot)
        gravity_tau = dynamics.Gravity(robot, q)
        tau = pd_tau + gravity_tau
        q_ddot = dynamics.ForwardDynamics(robot, q, q_dot, tau)

        q_dot += q_ddot * dt
        q += q_dot * dt

        fk = ForwardKinematics(robot, q)

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
        q_text = font.render(f"theta1 = {q[0]:.2f}, theta2 = {q[1]:.2f}", True, WHITE)
        q_dot_text = font.render(
            f"theta1_dot = {q_dot[0]:.2f}, theta2_dot = {q_dot[1]:.2f}", True, WHITE
        )
        tau_text = font.render(f"tau1 = {tau[0]:.2f}, tau2 = {tau[1]:.2f}", True, WHITE)
        difference_text = font.render(
            f"qd - q = ({(q_d[0] - q[0]):.2f}, {(q_d[1] - q[1]):.2f})", True, WHITE
        )

        if target is not None and mode == 1:
            target_screen = (
                BASE[0] + target[0] * SCALE,
                BASE[1] - target[1] * SCALE,
            )
            pygame.draw.circle(screen, RED, target_screen, 8)
        pygame.draw.aaline(screen, WHITE, BASE, J2)
        pygame.draw.aaline(screen, WHITE, J2, END)
        pygame.draw.circle(screen, RED, BASE, 8)
        pygame.draw.circle(screen, GREEN, J2, 8)
        pygame.draw.circle(screen, BLUE, END, 8)
        screen.blit(mode_text, (20, 0))
        screen.blit(gravity_text, (20, 20))
        screen.blit(q_text, (20, 60))
        screen.blit(q_dot_text, (20, 100))
        screen.blit(tau_text, (20, 140))
        screen.blit(difference_text, (20, 180))

        pygame.display.flip()

    pygame.quit()
    panel.close()


if __name__ == "__main__":
    main()
