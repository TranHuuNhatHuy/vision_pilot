"""Multiple vehicles driving in convoy and adjacent lanes on highway."""
import logging
from scenarios.PoV_Highway.base import PoVHighwayBase


class Scenario(PoVHighwayBase):
    scenario_name = "convoy_traffic"
    description = "Multiple vehicles driving in convoy and adjacent lanes (multi-target tracking)"

    def setup_npcs(self):
        # Lead vehicle 1: 35m ahead in same lane
        wp1 = self.get_waypoint_ahead(distance_m=35.0, lane="same")
        self.spawn_npc("vehicle.tesla.model3", wp1.transform, autopilot=True, speed_pct=25.0, keep_lane=True)

        # Lead vehicle 2: 75m ahead in same lane
        wp2 = self.get_waypoint_ahead(distance_m=75.0, lane="same")
        self.spawn_npc("vehicle.audi.a2", wp2.transform, autopilot=True, speed_pct=25.0, keep_lane=True)

        # Adjacent traffic: 20m ahead in left lane
        wp3 = self.get_waypoint_ahead(distance_m=20.0, lane="left")
        self.spawn_npc("vehicle.toyota.prius", wp3.transform, autopilot=True, speed_pct=15.0, keep_lane=True)

        logging.info("Spawned 3 convoy/adjacent vehicles")

