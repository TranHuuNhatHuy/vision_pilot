"""Base scenario definition for Town04."""
import carla
from scenarios.scenario_base import CarlaScenario


class Town04Base(CarlaScenario):
    """Base configuration for Town04 (Highway loop & small town)."""
    map_name = "Town04"
    scenario_name = "base"
    description = "Town04 baseline scenario"
    hero_type = "vehicle.lincoln.mkz_2020"
    hero_id = "hero"
    hero_spawn_index = 100
    weather = carla.WeatherParameters.ClearNoon

