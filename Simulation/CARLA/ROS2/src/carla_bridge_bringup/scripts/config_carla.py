#!/usr/bin/env python3
"""Spawn the VisionPilot ego + sensors + scenarios in CARLA via Python API.

Architecture: 1-Map-Multiple-Scenarios (Native CARLA Python API).
Each map folder under scenarios/ (e.g. scenarios/PoV_Highway, scenarios/Town04)
contains Python modules defining scenario classes (subclasses of CarlaScenario).

Backward compatibility: Legacy JSON scenario files can also be passed via -f.
"""

import argparse
import glob
import importlib
import importlib.util
import inspect
import json
import logging
import math
import os
import signal
import sys
import time

# Ensure scenarios package is in PYTHONPATH
_ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_scenarios_src = os.path.join(_ws_root, "scenarios")
if _scenarios_src not in sys.path:
    sys.path.insert(0, _ws_root)

# Also check installed share scenarios
_share_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _share_root not in sys.path:
    sys.path.insert(0, _share_root)

import carla
from scenarios.scenario_base import CarlaScenario


def find_scenarios_dir():
    """Locate the scenarios directory from workspace source tree or installed share."""
    candidates = [
        os.environ.get("CARLA_SCENARIOS_DIR", ""),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "scenarios")),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenarios")),
        os.path.abspath(os.path.join(os.getcwd(), "Simulation", "CARLA", "ROS2", "scenarios")),
        os.path.abspath(os.path.join(os.getcwd(), "scenarios")),
    ]
    for c in candidates:
        if c and os.path.isdir(c):
            return c
    return ""


def list_scenarios():
    """Discover and display all available maps and scenarios."""
    scenarios_dir = find_scenarios_dir()
    if not scenarios_dir or not os.path.isdir(scenarios_dir):
        print("Scenarios directory not found.")
        return

    print("==================================================================")
    print(" VisionPilot CARLA Simulation: Maps & Scenarios (Python API)")
    print(f" Root: {scenarios_dir}")
    print("==================================================================")

    maps = sorted([d for d in os.listdir(scenarios_dir) if os.path.isdir(os.path.join(scenarios_dir, d)) and not d.startswith("__")])
    if not maps:
        print(" No map scenario directories found.")
        return

    for m in maps:
        map_dir = os.path.join(scenarios_dir, m)
        print(f"\n Map: \033[1;36m{m}\033[0m")

        # Discover Python scenarios
        py_files = sorted(glob.glob(os.path.join(map_dir, "*.py")))
        for pf in py_files:
            fname = os.path.basename(pf)
            scen_id = os.path.splitext(fname)[0]
            if scen_id in ["__init__", "base"]:
                continue
            desc = "Python API Scenario"
            try:
                spec = importlib.util.spec_from_file_location(f"scenarios.{m}.{scen_id}", pf)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                if hasattr(mod, "Scenario"):
                    desc = getattr(mod.Scenario, "description", desc)
            except Exception as e:
                desc = f"(error inspecting: {e})"
            print(f"   * \033[1;32m{scen_id:<18}\033[0m: {desc}")

    print("\n------------------------------------------------------------------")
    print(" Launch commands:")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=lead_vehicle_acc")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=cut_in")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=static_obstacle")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=Town04 scenario:=dense_traffic")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=Town04 scenario:=foggy_drive")
    print("==================================================================")


def load_python_scenario_class(map_name, scenario_name):
    """Dynamically load Scenario class from scenarios.<map_name>.<scenario_name>."""
    scenarios_dir = find_scenarios_dir()
    if not scenarios_dir:
        raise RuntimeError("Scenarios directory not found.")

    module_file = os.path.join(scenarios_dir, map_name, f"{scenario_name}.py")
    if not os.path.isfile(module_file):
        return None

    spec = importlib.util.spec_from_file_location(f"scenarios.{map_name}.{scenario_name}", module_file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    if hasattr(mod, "Scenario"):
        return mod.Scenario
    raise AttributeError(f"Module {module_file} does not define a 'Scenario' class")


def create_scenario_from_json(json_path):
    """Adapt a legacy JSON file into a CarlaScenario subclass for backward compatibility."""
    with open(json_path) as f:
        data = json.load(f)

    # Check for base.json in same dir
    file_dir = os.path.abspath(os.path.dirname(json_path))
    base_path = os.path.join(file_dir, "base.json")
    if os.path.basename(json_path) != "base.json" and os.path.isfile(base_path):
        with open(base_path) as bf:
            base_data = json.load(bf)
        # Simple merge
        for k, v in base_data.items():
            if k not in data:
                data[k] = v

    class JsonAdaptedScenario(CarlaScenario):
        map_name = data.get("map", "Town04")
        scenario_name = data.get("scenario_name", os.path.splitext(os.path.basename(json_path))[0])
        description = data.get("description", "JSON Scenario")
        hero_type = data.get("hero", {}).get("type") or data.get("type", "vehicle.lincoln.mkz_2020")
        hero_id = data.get("hero", {}).get("id") or data.get("id", "hero")
        hero_spawn_index = int(data.get("hero", {}).get("spawn_index", data.get("spawn_index", 0)))

        def setup_npcs(self):
            for npc in data.get("npc_vehicles", []):
                bp = npc.get("type", "vehicle.*")
                if npc.get("spawn_relative_to") == "hero":
                    dist = float(npc.get("distance_m", 30.0))
                    lane = npc.get("lane", "same")
                    wp = self.get_waypoint_ahead(distance_m=dist, lane=lane)
                    self.spawn_npc(bp, wp.transform, autopilot=npc.get("autopilot", True),
                                   speed_pct=float(npc.get("speed_limit_percentage", 0.0)))
                elif npc.get("spawn_index") is not None:
                    sps = self.world.get_map().get_spawn_points()
                    idx = int(npc.get("spawn_index"))
                    if 0 <= idx < len(sps):
                        self.spawn_npc(bp, sps[idx], autopilot=npc.get("autopilot", True))

    return JsonAdaptedScenario


def resolve_scenario(file_arg="", map_arg="", scenario_arg=""):
    """Resolve scenario class from arguments."""
    target_map = map_arg if map_arg else "Town04"
    target_scen = scenario_arg if scenario_arg else ""

    # Case 1: An explicit JSON file path is given via -f
    if file_arg and os.path.isfile(file_arg):
        return create_scenario_from_json(file_arg)

    # Case 2: Shorthand passed via -f (e.g. PoV_Highway/lead_vehicle_acc)
    if file_arg:
        target = file_arg[:-3] if file_arg.endswith(".py") else file_arg
        target = target[:-5] if target.endswith(".json") else target
        if "/" in target:
            target_map, target_scen = target.split("/", 1)
        else:
            target_scen = target

    # Case 3: Specific map and scenario
    if target_scen:
        scen_cls = load_python_scenario_class(target_map, target_scen)
        if scen_cls is not None:
            return scen_cls

        # Fallback to json in scenarios dir
        s_dir = find_scenarios_dir()
        if s_dir:
            jp = os.path.join(s_dir, target_map, f"{target_scen}.json")
            if os.path.isfile(jp):
                return create_scenario_from_json(jp)

        raise FileNotFoundError(
            f"Scenario '{target_scen}' not found for map '{target_map}'. "
            f"Run with --list to see available scenarios."
        )

    # Case 4: Only map specified, pick default scenario
    default_candidates = ["free_drive", "free_cruise", "base"]
    for dc in default_candidates:
        scen_cls = load_python_scenario_class(target_map, dc)
        if scen_cls is not None:
            return scen_cls

    # Default fallback to Town04 free_cruise
    scen_cls = load_python_scenario_class("Town04", "free_cruise")
    if scen_cls is not None:
        return scen_cls

    # Legacy config/carla916.json
    legacy_json = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "config", "carla916.json"))
    if os.path.isfile(legacy_json):
        return create_scenario_from_json(legacy_json)

    raise RuntimeError("Could not resolve any scenario.")


def _setup_vehicle(world, scenario):
    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    candidates = bp_library.filter(scenario.hero_type)
    if not candidates:
        raise ValueError(f"No blueprint found for vehicle type '{scenario.hero_type}'")
    bp = candidates[0]
    bp.set_attribute("role_name", scenario.hero_id)
    bp.set_attribute("ros_name", scenario.hero_id)

    spawn_points = map_.get_spawn_points()
    default_idx = int(scenario.hero_spawn_index)
    idx = int(os.environ.get("SPAWN_INDEX", default_idx))

    if not 0 <= idx < len(spawn_points):
        raise IndexError(
            f"SPAWN_INDEX {idx} out of range: map {map_.name} has {len(spawn_points)} spawn points (valid 0..{len(spawn_points)-1})"
        )

    logging.info("Map %s: spawning hero vehicle at spawn index %d", map_.name, idx)
    spawn_pt = spawn_points[idx]
    return world.spawn_actor(bp, spawn_pt, attach_to=None)


def _setup_sensors(world, vehicle, sensors_config):
    bp_library = world.get_blueprint_library()
    sensors = []

    for sensor in sensors_config:
        logging.debug("Spawning sensor: %s", sensor.get("id"))
        bp = bp_library.filter(sensor.get("type"))[0]
        bp.set_attribute("ros_name", sensor.get("id"))
        bp.set_attribute("role_name", sensor.get("id"))
        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        wp = carla.Transform(
            location=carla.Location(
                x=sensor["spawn_point"]["x"],
                y=-sensor["spawn_point"]["y"],
                z=sensor["spawn_point"]["z"],
            ),
            rotation=carla.Rotation(
                roll=sensor["spawn_point"]["roll"],
                pitch=-sensor["spawn_point"]["pitch"],
                yaw=-sensor["spawn_point"]["yaw"],
            ),
        )

        sensor_actor = world.spawn_actor(bp, wp, attach_to=vehicle)
        sensor_actor.enable_for_ros()
        sensors.append(sensor_actor)

    return sensors


def _follow_vehicle(world, vehicle, spectator):
    vehicle_transform = vehicle.get_transform()
    location = vehicle_transform.location
    rotation = vehicle_transform.rotation

    # Chase camera: 6m behind, 2.5m above
    offset_distance = 6.0
    height = 2.5

    yaw_rad = math.radians(rotation.yaw)
    dx = -offset_distance * math.cos(yaw_rad)
    dy = -offset_distance * math.sin(yaw_rad)

    offset_location = carla.Location(x=location.x + dx, y=location.y + dy, z=location.z + height)
    spectator.set_transform(carla.Transform(offset_location, rotation))


def main(args):
    if args.list:
        list_scenarios()
        return

    world = None
    vehicle = None
    sensors = []
    scenario_instance = None
    original_settings = None

    try:
        ScenarioClass = resolve_scenario(
            file_arg=args.file,
            map_arg=args.map,
            scenario_arg=args.scenario,
        )

        map_name = args.map if args.map else getattr(ScenarioClass, "map_name", "Town04")
        scen_name = getattr(ScenarioClass, "scenario_name", "custom")
        desc = getattr(ScenarioClass, "description", "")

        logging.info("Starting Scenario: [%s] on Map [%s]", scen_name, map_name)
        if desc:
            logging.info("Scenario Description: %s", desc)

        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)

        current_world_name = client.get_world().get_map().name
        if current_world_name != map_name and not current_world_name.endswith("/" + map_name):
            logging.info("Loading map: %s", map_name)
            client.load_world(map_name)
        else:
            logging.info("Map '%s' is already active", map_name)

        world = client.get_world()

        # Synchronous mode configuration
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1  # 10 Hz simulation step
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # Apply weather
        if hasattr(ScenarioClass, "weather") and ScenarioClass.weather is not None:
            world.set_weather(ScenarioClass.weather)
            logging.info("Applied weather parameters to world")

        # 1. Spawn ego vehicle and mount sensors
        vehicle = _setup_vehicle(world, ScenarioClass)
        sensors = _setup_sensors(world, vehicle, ScenarioClass.sensors_config)

        # 2. CRITICAL STEP: Tick once to commit ego vehicle physics to server!
        world.tick()

        hero_loc = vehicle.get_location()
        logging.info("Hero vehicle active at (%0.2f, %0.2f, %0.2f)", hero_loc.x, hero_loc.y, hero_loc.z)

        # 3. Instantiate scenario and spawn NPCs relative to true hero position
        scenario_instance = ScenarioClass(world, traffic_manager, hero_vehicle=vehicle)
        scenario_instance.setup_npcs()

        # Tick to settle NPC vehicle physics
        world.tick()
        logging.info("Spawned %d scenario NPC/obstacle actors", len(scenario_instance.npc_actors) + len(scenario_instance.static_actors))

        if args.autopilot:
            vehicle.set_autopilot(True)

        spectator = world.get_spectator()
        logging.info("Running... (ego up; telemetry is published by the bridge container)")

        TARGET_HZ = 10.0
        TARGET_PERIOD = 1.0 / TARGET_HZ

        while True:
            loop_start = time.time()

            world.tick()
            if scenario_instance:
                scenario_instance.tick()
            _follow_vehicle(world, vehicle, spectator)

            elapsed = time.time() - loop_start
            sleep_time = TARGET_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nCancelled by user. Shutting down cleanly...")

    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if scenario_instance:
                scenario_instance.cleanup()

            for sensor in sensors:
                try:
                    if sensor.is_alive:
                        sensor.destroy()
                except Exception as e:
                    logging.debug("Error destroying sensor: %s", e)

            if vehicle:
                try:
                    if vehicle.is_alive:
                        vehicle.set_autopilot(False)
                        vehicle.destroy()
                except Exception as e:
                    logging.debug("Error destroying vehicle: %s", e)

            if world:
                try:
                    world.tick()
                except Exception:
                    pass

            if traffic_manager:
                try:
                    traffic_manager.set_synchronous_mode(False)
                except Exception:
                    pass

            if original_settings and world:
                logging.info("Restoring original settings")
                try:
                    world.apply_settings(original_settings)
                except Exception as e:
                    logging.debug("Error restoring settings: %s", e)

        finally:
            signal.signal(signal.SIGINT, signal.default_int_handler)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA Python API Scenario Runner for VisionPilot")
    argparser.add_argument("--host", default="localhost", help="IP of CARLA Simulator (default: localhost)")
    argparser.add_argument("--port", default=2000, type=int, help="TCP port of CARLA Simulator (default: 2000)")
    argparser.add_argument("-f", "--file", default="", help="Direct path to Python or JSON scenario file")
    argparser.add_argument("-m", "--map", type=str, default="", help="CARLA map name (e.g. PoV_Highway, Town04)")
    argparser.add_argument("-s", "--scenario", type=str, default="", help="Scenario name (e.g. lead_vehicle_acc, cut_in, convoy_traffic, static_obstacle, dense_traffic)")
    argparser.add_argument("-l", "--list", action="store_true", help="List all available maps and scenarios")
    argparser.add_argument("-v", "--verbose", action="store_true", dest="debug", help="Print debug information")
    argparser.add_argument("-a", "--autopilot", action="store_true", help="Turn on autopilot for hero vehicle")

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    main(args)