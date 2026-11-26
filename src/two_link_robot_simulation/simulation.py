import numpy as np
import pygame

from . import dynamics
from .controller import CONTROL_MODES, PD_GRAVITY, PDController, wrap_angle
from .integrator import DEFAULT_TORQUE_LIMITS, MAX_FRAME_DT, step
from .kinematics import InverseKinematics
from .pygame_view import BASE, SCALE, initialize, render
from .robot import Link, Robot
from .tkinter_controls import ControlPanel
from .trajectory import QuinticTrajectory, plan

FRAMES_PER_SECOND = 60

KP = 30
KD = 30

# Goal changes smaller than this (rad) do not trigger a re-plan.
REPLAN_TOLERANCE = 1e-3


def event_handler(
    robot: Robot,
    panel: ControlPanel,
    mode: int,
    target: np.ndarray | None,
    q_goal: np.ndarray,
    reachable: bool,
    running: bool,
    control_mode: str,
    torque_limited: bool,
):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False

            elif event.key == pygame.K_g:
                dynamics.g = 0 if dynamics.g != 0 else 9.81

            elif event.key == pygame.K_f:
                index = CONTROL_MODES.index(control_mode)
                control_mode = CONTROL_MODES[(index + 1) % len(CONTROL_MODES)]

            elif event.key == pygame.K_l:
                torque_limited = not torque_limited

            elif event.key == pygame.K_TAB:
                mode = 2 if mode == 1 else 1

                if mode == 2:
                    # Start the sliders at the current goal, otherwise the
                    # arm would jump to whatever the sliders last showed.
                    goal = wrap_angle(q_goal)
                    panel.set_angles(goal[0], goal[1])
                    panel.show()
                else:
                    panel.hide()

        elif event.type == pygame.MOUSEBUTTONDOWN and mode == 1:
            mouse_x, mouse_y = event.pos

            x = (mouse_x - BASE[0]) / SCALE
            y = (BASE[1] - mouse_y) / SCALE

            target = np.array([x, y])
            new_q_goal = InverseKinematics(robot, target)

            if new_q_goal is not None:
                q_goal = new_q_goal
                reachable = True
            else:
                reachable = False

    return mode, target, q_goal, reachable, running, control_mode, torque_limited


def replan_if_needed(
    trajectory: QuinticTrajectory,
    q_goal: np.ndarray,
    t: float,
    max_velocity: float,
) -> QuinticTrajectory:
    """Start a new move when the goal changes.

    The new move starts from the current reference state (not the measured
    one), so the reference stays continuous in position, velocity and
    acceleration and the commanded torque does not jump.
    """
    if np.max(np.abs(wrap_angle(q_goal - trajectory.q_end))) < REPLAN_TOLERANCE:
        return trajectory

    q_ref, q_dot_ref, q_ddot_ref = trajectory.sample(t)

    return plan(
        q_ref,
        q_goal,
        t,
        v_start=q_dot_ref,
        a_start=q_ddot_ref,
        max_velocity=max_velocity,
    )


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

    q_goal = np.zeros(2)
    t = 0.0
    trajectory = plan(q, q_goal, t)

    control_mode = PD_GRAVITY
    torque_limited = False

    target = None
    reachable = False
    mode = 1
    running = True

    while running and panel.running:
        panel.root.update()

        # The Tk window may have been closed inside update().
        if not panel.running:
            break

        (
            mode,
            target,
            q_goal,
            reachable,
            running,
            control_mode,
            torque_limited,
        ) = event_handler(
            robot,
            panel,
            mode,
            target,
            q_goal,
            reachable,
            running,
            control_mode,
            torque_limited,
        )

        if mode == 2:
            q_goal = np.array(
                [
                    panel.theta1.get(),
                    panel.theta2.get(),
                ]
            )

            controller.kp = int(panel.kp.get())
            controller.kd = int(panel.kd.get())

        trajectory = replan_if_needed(
            trajectory,
            q_goal,
            t,
            panel.max_velocity.get(),
        )

        dt = min(clock.tick(FRAMES_PER_SECOND) / 1000, MAX_FRAME_DT)

        q, q_dot, tau = step(
            robot,
            controller,
            q,
            q_dot,
            q_goal,
            np.zeros(2),
            dt,
            trajectory=trajectory,
            t=t,
            mode=control_mode,
            torque_limits=DEFAULT_TORQUE_LIMITS if torque_limited else None,
        )

        t += dt

        q_ref, _, _ = trajectory.sample(t)

        render(
            robot,
            q,
            q_goal,
            q_dot,
            tau,
            target,
            reachable,
            mode,
            q_ref=q_ref,
            trajectory=trajectory,
            t=t,
            control_mode=control_mode,
            torque_limited=torque_limited,
        )

    pygame.quit()
    panel.close()


if __name__ == "__main__":
    simulation()
