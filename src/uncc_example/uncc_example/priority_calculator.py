import math

from .models import Frontier


class PriorityCalculator:
    def __init__(
        self,
        information_gain_weight: float = 2.0,
        mission_value_weight: float = 1.5,
        path_cost_weight: float = 0.8,
        hazard_weight: float = 2.0,
        global_cost_weight: float = 0.5,
        narrowness_weight: float = 0.5,
        failure_weight: float = 2.0,
        info_reference_m: float = 2.0,
        path_reference_m: float = 8.0,
    ):
        self.w_info = float(information_gain_weight)
        self.w_mission = float(mission_value_weight)
        self.w_path = float(path_cost_weight)
        self.w_hazard = float(hazard_weight)
        self.w_global = float(global_cost_weight)
        self.w_narrow = float(narrowness_weight)
        self.w_failure = float(failure_weight)

        self.info_reference_m = max(0.01, float(info_reference_m))
        self.path_reference_m = max(0.01, float(path_reference_m))

    @staticmethod
    def _clip01(value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    def calculate_score(self, frontier: Frontier) -> float:
        info = self._clip01(
            frontier.information_gain / self.info_reference_m
        )

        if math.isfinite(frontier.path_length):
            path = self._clip01(
                frontier.path_length / self.path_reference_m
            )
        else:
            path = 1.0

        mission = self._clip01(frontier.mission_value)
        hazard = self._clip01(frontier.hazard_risk)
        global_cost = self._clip01(frontier.global_cost)
        narrowness = self._clip01(frontier.narrowness)
        failure = self._clip01(frontier.failure_penalty)

        frontier.score = (
            self.w_info * info
            + self.w_mission * mission
            - self.w_path * path
            - self.w_hazard * hazard
            - self.w_global * global_cost
            - self.w_narrow * narrowness
            - self.w_failure * failure
        )

        return frontier.score
