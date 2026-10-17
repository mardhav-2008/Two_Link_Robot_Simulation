class Link:
    def __init__(self, length: float = 1, angle: float = 0, mass: float = 5) -> None:
        self.length = length
        self.angle = angle
        self.mass = mass

    @property
    def inertia(self):
        return (1 / 12) * self.mass * self.length**2

    @property
    def com(self):
        return self.length / 2


class Joint:
    def __init__(self, link1: Link, link2: Link):
        self.link1 = link1
        self.link2 = link2
        self.angle = self.link2.angle - self.link1.angle


class Robot:
    def __init__(self, links: list[Link]) -> None:
        self.links = links
        self.joints = [
            Joint(self.links[i], self.links[i + 1]) for i in range(len(self.links) - 1)
        ]
