"""Module A: maze generation for Gazebo, with adjustable parameters. Pure Python (ANR1)."""

from .generator import GenerationError, Maze, derive_seed, generate  # noqa: F401
from .mazeset import generate_set, load_maze_meta, maze_name  # noqa: F401
from .params import MazeParams, max_exits, valid_corridor_widths, validate  # noqa: F401
