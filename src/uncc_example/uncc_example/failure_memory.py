import math
import time
from dataclasses import dataclass
from typing import List


@dataclass
class FailedGoal:
    x: float
    y: float
    stamp: float


class FailureMemory:
    def __init__(
        self,
        radius: float = 0.60,
        ttl: float = 120.0,
    ):
        self.radius = float(radius)
        self.ttl = float(ttl)
        self.failed: List[FailedGoal] = []

    def _purge(self):
        if self.ttl <= 0:
            return

        now = time.monotonic()
        self.failed = [
            item
            for item in self.failed
            if now - item.stamp <= self.ttl
        ]

    def record(self, x: float, y: float):
        self._purge()
        self.failed.append(
            FailedGoal(
                x=float(x),
                y=float(y),
                stamp=time.monotonic(),
            )
        )

    def penalty(self, x: float, y: float) -> float:
        self._purge()

        for item in self.failed:
            if math.hypot(x - item.x, y - item.y) <= self.radius:
                return 1.0

        return 0.0
