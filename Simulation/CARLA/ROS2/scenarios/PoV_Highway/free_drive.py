"""Free driving on PoV_Highway with clear weather and no other vehicles."""
from scenarios.PoV_Highway.base import PoVHighwayBase


class Scenario(PoVHighwayBase):
    scenario_name = "free_drive"
    description = "Free driving on open highway with clear weather and no obstacles (LKAS test)"

    def setup_npcs(self):
        # Empty highway for baseline lane keeping tests
        pass

