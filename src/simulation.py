import numpy as np
import pygame

HEIGHT = 720
WIDTH = 1080
FRAMES_PER_SECOND = 120
SCALE = 200

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

PI = np.pi
BASE = (WIDTH / 2, HEIGHT / 2)


pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

running = True
