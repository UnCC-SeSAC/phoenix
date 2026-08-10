import math
from dataclasses import dataclass
from typing import Optional, Tuple

from nav_msgs.msg import OccupancyGrid, Path


@dataclass
class GridMap:
    frame_id: str
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: list

    @classmethod
    def from_msg(cls, msg: OccupancyGrid):
        return cls(
            frame_id=msg.header.frame_id,
            resolution=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            data=list(msg.data),
        )

    def valid(self, mx: int, my: int) -> bool:
        return 0 <= mx < self.width and 0 <= my < self.height

    def index(self, mx: int, my: int) -> int:
        return my * self.width + mx

    def get(self, mx: int, my: int, default: int = -1) -> int:
        if not self.valid(mx, my):
            return default
        return int(self.data[self.index(mx, my)])

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        mx = int(math.floor((x - self.origin_x) / self.resolution))
        my = int(math.floor((y - self.origin_y) / self.resolution))
        if not self.valid(mx, my):
            return None
        return mx, my

    def cell_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        x = self.origin_x + (mx + 0.5) * self.resolution
        y = self.origin_y + (my + 0.5) * self.resolution
        return x, y

    def normalized_cost_at_world(
        self,
        x: float,
        y: float,
        unknown_cost: float = 0.70,
    ) -> float:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return 1.0

        value = self.get(*cell)
        if value < 0:
            return float(unknown_cost)

        return max(0.0, min(1.0, value / 100.0))

    def max_cost_in_radius(
        self,
        x: float,
        y: float,
        radius: float,
        unknown_cost: float = 0.70,
    ) -> float:
        center = self.world_to_cell(x, y)
        if center is None:
            return 1.0

        cx, cy = center
        radius_cells = max(1, int(math.ceil(radius / self.resolution)))
        max_cost = 0.0

        for my in range(cy - radius_cells, cy + radius_cells + 1):
            for mx in range(cx - radius_cells, cx + radius_cells + 1):
                if not self.valid(mx, my):
                    continue

                wx, wy = self.cell_to_world(mx, my)
                if math.hypot(wx - x, wy - y) > radius:
                    continue

                value = self.get(mx, my)
                cost = unknown_cost if value < 0 else value / 100.0
                max_cost = max(max_cost, cost)

        return max(0.0, min(1.0, max_cost))

    def occupied_ratio_in_radius(
        self,
        x: float,
        y: float,
        radius: float,
        occupied_threshold: int = 65,
    ) -> float:
        center = self.world_to_cell(x, y)
        if center is None:
            return 1.0

        cx, cy = center
        radius_cells = max(1, int(math.ceil(radius / self.resolution)))

        total = 0
        occupied = 0

        for my in range(cy - radius_cells, cy + radius_cells + 1):
            for mx in range(cx - radius_cells, cx + radius_cells + 1):
                if not self.valid(mx, my):
                    continue

                wx, wy = self.cell_to_world(mx, my)
                if math.hypot(wx - x, wy - y) > radius:
                    continue

                value = self.get(mx, my)
                if value < 0:
                    continue

                total += 1
                if value >= occupied_threshold:
                    occupied += 1

        if total == 0:
            return 1.0
        return occupied / total


def path_length(path: Path) -> float:
    if path is None or len(path.poses) < 2:
        return math.inf

    total = 0.0
    prev = path.poses[0].pose.position

    for stamped in path.poses[1:]:
        cur = stamped.pose.position
        total += math.hypot(cur.x - prev.x, cur.y - prev.y)
        prev = cur

    return total


def path_average_cost(
    path: Path,
    grid: Optional[GridMap],
    unknown_cost: float = 0.70,
) -> float:
    if grid is None or path is None or len(path.poses) == 0:
        return 0.0

    values = [
        grid.normalized_cost_at_world(
            p.pose.position.x,
            p.pose.position.y,
            unknown_cost=unknown_cost,
        )
        for p in path.poses
    ]
    return sum(values) / len(values)


def path_max_cost(
    path: Path,
    grid: Optional[GridMap],
    unknown_cost: float = 0.70,
) -> float:
    if grid is None or path is None or len(path.poses) == 0:
        return 0.0

    return max(
        grid.normalized_cost_at_world(
            p.pose.position.x,
            p.pose.position.y,
            unknown_cost=unknown_cost,
        )
        for p in path.poses
    )
