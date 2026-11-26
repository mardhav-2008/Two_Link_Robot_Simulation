from .controller import PDController
from .kinematics import ForwardKinematics, InverseKinematics
from .robot import Joint, Link, Robot

__version__ = "0.1.0"


def main() -> None:
    """Entry point for the `two-link-robot-simulation` command.

    pygame and tkinter are imported here, not at package import, so the
    physics modules stay usable (and testable) without a display.
    """
    from .simulation import simulation

    simulation()
