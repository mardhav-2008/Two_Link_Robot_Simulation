import numpy as np
import pygame

from kinematics import ForwardKinematics

HEIGHT = 720
WIDTH = 1080
FRAMES_PER_SECOND = 60
SCALE = 200

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

PI = np.pi
BASE = (WIDTH / 2, HEIGHT / 2)


class Link:
    def __init__(self, length: float = 1, angle: float = 0):
        self.length = length
        self.angle = angle


class Joint:
    def __init__(self, Link1: Link, Link2: Link):
        self.Link1 = Link1
        self.Link2 = Link2
        self.angle = Link2.angle - Link1.angle

    def FK(self, angle):
        fk = ForwardKinematics(
            self.Link1.length, self.Link2.length, self.Link1.angle, angle
        )

        self.angle = angle
        self.Link2.angle = self.Link1.angle + angle

        return fk


pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

running = True
link1 = Link()
link2 = Link()
joint1 = Joint(link1, link2)

theta = 0
dt = 0
time = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (
            event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE)
        ):
            running = False

    screen.fill(BLACK)

    fk = joint1.FK(theta + PI * time)

    J2 = (BASE[0] + fk[0][0] * SCALE, BASE[1] - fk[0][1] * SCALE)

    END = (BASE[0] + fk[1][0] * SCALE, BASE[1] - fk[1][1] * SCALE)

    pygame.draw.line(screen, WHITE, BASE, J2, 5)
    pygame.draw.line(screen, WHITE, J2, END, 5)

    pygame.display.flip()

    dt = clock.tick(FRAMES_PER_SECOND) / 1000
    time += dt

pygame.quit()
