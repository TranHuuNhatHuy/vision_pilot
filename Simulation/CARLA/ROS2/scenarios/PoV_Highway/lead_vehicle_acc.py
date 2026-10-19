"""Hero vehicle follows a lead vehicle cruising in-lane (ACC following test)."""
import logging
from scenarios.PoV_Highway.base import PoVHighwayBase


class Scenario(PoVHighwayBase):
    scenario_name = "lead_vehicle_acc"
    description = "Hero vehicle follows a lead vehicle cruising in-lane (ACC following & FCW test)"

    def setup_npcs(self):
        # Place lead vehicle 45 meters ahead in the same lane
        wp = self.get_waypoint_ahead(distance_m=45.0, lane="same")
        self.lead_vehicle = self.spawn_npc(
            "vehicle.tesla.model3",
            wp.transform,
            autopilot=True,
            speed_pct=20.0,
            keep_lane=True,
        )
        if self.lead_vehicle:
            logging.info("Lead vehicle spawned 45m ahead in same lane")

