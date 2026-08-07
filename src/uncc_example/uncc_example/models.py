from dataclasses import dataclass, field
from typing import List, Tuple
import math


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Frontier:
    frontier_id: int
    cells: List[Tuple[int, int]]
    x: float
    y: float
    size: int
    information_gain: float

    euclidean_distance: float = math.inf
    path_length: float = math.inf

    global_cost: float = 0.0
    hazard_risk: float = 0.0
    narrowness: float = 0.0
    mission_value: float = 0.0
    failure_penalty: float = 0.0

    score: float = -math.inf

    def reachable(self) -> bool:
        return math.isfinite(self.path_length)
