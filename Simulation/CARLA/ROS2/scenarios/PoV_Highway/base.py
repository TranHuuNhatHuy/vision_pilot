"""Base scenario definition for PoV_Highway."""
import carla
from scenarios.scenario_base import CarlaScenario


class PoVHighwayBase(CarlaScenario):
    """Base configuration for PoV_Highway (Autoware Simulation WG 15km Highway)."""
    map_name = "PoV_Highway"
    scenario_name = "base"
    description = "PoV_Highway baseline scenario"
    hero_type = "vehicle.lincoln.mkz_2020"
    hero_id = "hero"
    hero_spawn_index = 0
    weather = carla.WeatherParameters.ClearNoon

