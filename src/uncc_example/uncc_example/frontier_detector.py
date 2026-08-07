from collections import deque
import math
from typing import List, Set, Tuple

from .map_utils import GridMap
from .models import Frontier


class FrontierDetector:
    """
    Detect frontiers from the raw SLAM OccupancyGrid.

    A frontier cell is:
      1) a FREE cell, and
      2) adjacent to at least one UNKNOWN cell.

    Frontier detection intentionally uses /map rather than a Nav2 costmap.
    """

    FOUR_NEIGHBORS = (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
    )

    EIGHT_NEIGHBORS = (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    )

    def __init__(
        self,
        free_threshold: int = 20,
        min_cluster_size: int = 8,
    ):
        self.free_threshold = int(free_threshold)
        self.min_cluster_size = int(min_cluster_size)

    def _is_free(self, value: int) -> bool:
        return 0 <= value <= self.free_threshold

    def _is_frontier_cell(self, grid: GridMap, mx: int, my: int) -> bool:
        if not self._is_free(grid.get(mx, my)):
            return False

        for dx, dy in self.FOUR_NEIGHBORS:
            nx = mx + dx
            ny = my + dy
            if grid.valid(nx, ny) and grid.get(nx, ny) == -1:
                return True

        return False

    def _frontier_cells(self, grid: GridMap) -> Set[Tuple[int, int]]:
        cells = set()

        for my in range(1, grid.height - 1):
            for mx in range(1, grid.width - 1):
                if self._is_frontier_cell(grid, mx, my):
                    cells.add((mx, my))

        return cells

    def _cluster(
        self,
        start: Tuple[int, int],
        frontier_cells: Set[Tuple[int, int]],
        visited: Set[Tuple[int, int]],
    ) -> List[Tuple[int, int]]:
        q = deque([start])
        visited.add(start)
        cluster = []

        while q:
            cell = q.popleft()
            cluster.append(cell)

            cx, cy = cell
            for dx, dy in self.EIGHT_NEIGHBORS:
                nxt = (cx + dx, cy + dy)
                if nxt in frontier_cells and nxt not in visited:
                    visited.add(nxt)
                    q.append(nxt)

        return cluster

    def _representative_cell(
        self,
        cells: List[Tuple[int, int]],
    ) -> Tuple[int, int]:
        mean_x = sum(c[0] for c in cells) / len(cells)
        mean_y = sum(c[1] for c in cells) / len(cells)

        return min(
            cells,
            key=lambda c: math.hypot(c[0] - mean_x, c[1] - mean_y),
        )

    def find_frontiers(
        self,
        grid: GridMap,
        max_frontiers: int = 100,
    ) -> List[Frontier]:
        frontier_cells = self._frontier_cells(grid)
        visited = set()
        frontiers = []
        frontier_id = 0

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = self._cluster(cell, frontier_cells, visited)

            if len(cluster) < self.min_cluster_size:
                continue

            rep_x, rep_y = self._representative_cell(cluster)
            world_x, world_y = grid.cell_to_world(rep_x, rep_y)

            # Approximate useful boundary length in metres.
            information_gain = len(cluster) * grid.resolution

            frontiers.append(
                Frontier(
                    frontier_id=frontier_id,
                    cells=cluster,
                    x=world_x,
                    y=world_y,
                    size=len(cluster),
                    information_gain=information_gain,
                )
            )
            frontier_id += 1

        frontiers.sort(key=lambda f: f.size, reverse=True)
        return frontiers[:max_frontiers]
