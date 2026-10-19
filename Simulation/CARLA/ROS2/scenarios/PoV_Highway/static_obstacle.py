"""Stationary stalled vehicle in lane ahead for Forward Collision Warning & AEB test."""
import logging
import carla
from scenarios.PoV_Highway.base import PoVHighwayBase


class Scenario(PoVHighwayBase):
    scenario_name = "static_obstacle"
    description = "Stationary stalled vehicle in lane ahead (AEB / FCW obstacle detection test)"

    def setup_npcs(self):
        # Place stationary hazard 50m directly ahead in the same lane
        wp = self.get_waypoint_ahead(distance_m=50.0, lane="same")
        stalled = self.spawn_npc("vehicle.audi.a2", wp.transform, autopilot=False)
        if stalled:
            try:
                # Turn on hazard / warning lights
                stalled.set_light_state(carla.VehicleLightState(
                    carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2
                ))
            except Exception:
                pass
            logging.info("Spawned stationary hazard vehicle 50m ahead with hazard lights")

