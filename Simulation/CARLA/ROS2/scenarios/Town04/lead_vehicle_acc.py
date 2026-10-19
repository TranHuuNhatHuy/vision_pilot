"""In-lane lead vehicle following on Town04 highway loop."""
import logging
from scenarios.Town04.base import Town04Base


class Scenario(Town04Base):
    scenario_name = "lead_vehicle_acc"
    description = "Hero vehicle follows a lead vehicle cruising on Town04 highway loop (ACC test)"

    def setup_npcs(self):
        wp = self.get_waypoint_ahead(distance_m=45.0, lane="same")
        self.spawn_npc(
            "vehicle.tesla.model3",
            wp.transform,
            autopilot=True,
            speed_pct=20.0,
            keep_lane=True,
        )
        logging.info("Spawned lead vehicle 45m ahead on Town04 loop")

