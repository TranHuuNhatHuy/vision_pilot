"""Adverse weather challenge: low visibility fog on highway."""
import logging
import carla
from scenarios.PoV_Highway.base import PoVHighwayBase


class Scenario(PoVHighwayBase):
    scenario_name = "foggy_drive"
    description = "Adverse weather challenge: dense fog and low visibility on highway"

    weather = carla.WeatherParameters(
        cloudiness=90.0,
        precipitation=10.0,
        precipitation_deposits=40.0,
        wind_intensity=10.0,
        sun_azimuth_angle=180.0,
        sun_altitude_angle=15.0,
        fog_density=75.0,
        fog_distance=15.0,
        fog_falloff=1.5,
        wetness=50.0,
    )

    def setup_npcs(self):
        # Place lead vehicle 40m ahead in lane
        wp = self.get_waypoint_ahead(distance_m=40.0, lane="same")
        self.spawn_npc(
            "vehicle.tesla.model3",
            wp.transform,
            autopilot=True,
            speed_pct=25.0,
            keep_lane=True,
        )
        logging.info("Spawned lead vehicle for foggy drive scenario on PoV_Highway")

