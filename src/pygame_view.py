import pygame

HEIGHT = 720
WIDTH = 1080
FRAMES_PER_SECOND = 120
SCALE = 150

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

BASE = (WIDTH / 2, HEIGHT / 2)


class PygameView:
    def __init__(self):
        pygame.init()

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)

    def handle_events(self):
        events = pygame.event.get()

        for event in events:
            if event.type == pygame.QUIT:
                return False

        return True

    def draw(self):
        self.screen.fill(BLACK)
        pygame.display.flip()

    def tick(self):
        return self.clock.tick(FRAMES_PER_SECOND) / 1000

    def close(self):
        pygame.quit()
