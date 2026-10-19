"""Dense fog and adverse weather challenge on Town04."""
import logging
import carla
from scenarios.Town04.base import Town04Base


class Scenario(Town04Base):
    scenario_name = "foggy_drive"
    description = "Adverse weather challenge: dense fog and low visibility on Town04 highway"

    # Town04 contains BP_Weather so dynamic weather parameters render fully
    weather = carla.WeatherParameters(
        cloudiness=90.0,
        precipitation=15.0,
        precipitation_deposits=50.0,
        wind_intensity=10.0,
        sun_azimuth_angle=180.0,
        sun_altitude_angle=15.0,
        fog_density=80.0,
        fog_distance=10.0,
        fog_falloff=1.5,
        wetness=60.0,
    )

    def setup_npcs(self):
        # Lead vehicle driving in the fog 35m ahead
        wp = self.get_waypoint_ahead(distance_m=35.0, lane="same")
        self.spawn_npc(
            "vehicle.tesla.model3",
            wp.transform,
            autopilot=True,
            speed_pct=25.0,
            keep_lane=True,
        )
        logging.info("Spawned lead vehicle in dense fog on Town04")

