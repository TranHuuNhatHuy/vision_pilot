"""Cut-in vehicle scenario on Town04."""
import logging
from scenarios.Town04.base import Town04Base


class Scenario(Town04Base):
    scenario_name = "cut_in"
    description = "Adjacent vehicle executes an active cut-in maneuver on Town04 highway"

    def setup_npcs(self):
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

    def tick(self):
        self.ticks_count += 1
        if not self.cut_in_triggered and self.cut_in_vehicle and self.cut_in_vehicle.is_alive:
            if self.ticks_count > 50:
                logging.info("Triggering cut-in maneuver into hero lane on Town04!")
                self.traffic_manager.force_lane_change(self.cut_in_vehicle, True)
                self.cut_in_triggered = True

