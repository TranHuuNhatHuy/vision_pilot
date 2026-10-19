"""Free cruise on Town04 highway loop with clear weather and no traffic."""
from scenarios.Town04.base import Town04Base


class Scenario(Town04Base):
    scenario_name = "free_cruise"
    description = "Town04 highway loop cruise with clear weather and no traffic"

    def setup_npcs(self):
        # Empty highway loop for continuous endurance testing
        pass

