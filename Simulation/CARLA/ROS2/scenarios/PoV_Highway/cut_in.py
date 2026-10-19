"""Adjacent lane vehicle performs an active cut-in maneuver ahead of hero vehicle."""
import logging
from scenarios.PoV_Highway.base import PoVHighwayBase


class Scenario(PoVHighwayBase):
    scenario_name = "cut_in"
    description = "Adjacent vehicle executes an active cut-in maneuver ahead of hero vehicle"

    def setup_npcs(self):
        # Place cut-in vehicle 25 meters ahead in the left lane
        wp = self.get_waypoint_ahead(distance_m=25.0, lane="left")
        self.cut_in_vehicle = self.spawn_npc(
            "vehicle.audi.a2",
            wp.transform,
            autopilot=True,
            speed_pct=10.0,
            keep_lane=True,
        )
        self.cut_in_triggered = False
        self.ticks_count = 0
        if self.cut_in_vehicle:
            logging.info("Cut-in vehicle spawned 25m ahead in left lane")

    def tick(self):
        self.ticks_count += 1
        # Trigger cut-in after simulation has stabilized (e.g. 50 ticks = 5 seconds)
        # or when hero vehicle closes distance
        if not self.cut_in_triggered and self.cut_in_vehicle and self.cut_in_vehicle.is_alive:
            if self.ticks_count > 50:
                logging.info("Triggering cut-in maneuver into hero lane!")
                # force_lane_change(actor, direction): True = Right, False = Left
                # Since cut-in vehicle is in left lane, changing to right enters hero lane
                self.traffic_manager.force_lane_change(self.cut_in_vehicle, True)
                self.cut_in_triggered = True

