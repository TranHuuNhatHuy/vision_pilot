"""Dense ambient highway traffic on Town04."""
import logging
import random
from scenarios.Town04.base import Town04Base


class Scenario(Town04Base):
    scenario_name = "dense_traffic"
    description = "Town04 highway loop with multiple vehicles and ambient traffic"

    def setup_npcs(self):
        spawn_points = self.world.get_map().get_spawn_points()
        # Highway NPCs at fixed points
        if len(spawn_points) > 152:
            self.spawn_npc("vehicle.audi.a2", spawn_points[151], autopilot=True)
            self.spawn_npc("vehicle.tesla.model3", spawn_points[152], autopilot=True)

        # Ambient random vehicles
        free = [p for i, p in enumerate(spawn_points) if i not in [100, 151, 152]]
        random.shuffle(free)
        for sp in free[:10]:
            self.spawn_npc("vehicle.*", sp, autopilot=True)

        logging.info("Spawned dense ambient traffic on Town04")

