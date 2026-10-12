#!/usr/bin/env python3
"""Spawn the VisionPilot ego + sensors + NPC traffic in CARLA.

Architecture: 1-map-multiple-scenarios.
Each map folder under scenarios/ (e.g., scenarios/PoV_Highway, scenarios/Town04)
contains:
  - base.json: defines the base vehicle, default sensors, default weather, and map name.
  - <scenario>.json: defines scenario-specific overrides (NPC traffic, weather challenges, spawn points).

Pure CARLA PythonAPI — no ROS dependency inside this script.
Native CARLA --ros2 publishes camera and sensors directly.
"""

import argparse
import glob
import json
import logging
import math
import os
import random
import signal
import sys
import time

import carla


def find_scenarios_dir():
    """Locate the scenarios directory from source tree, installed share, or environment."""
    candidates = [
        os.environ.get("CARLA_SCENARIOS_DIR", ""),
        # Installed package share: <prefix>/share/carla_bridge_bringup/scenarios
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scenarios")),
        # Workspace source tree: <ws>/Simulation/CARLA/ROS2/scenarios
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "scenarios")),
        # Relative to cwd
        os.path.abspath(os.path.join(os.getcwd(), "Simulation", "CARLA", "ROS2", "scenarios")),
        os.path.abspath(os.path.join(os.getcwd(), "scenarios")),
    ]
    for c in candidates:
        if c and os.path.isdir(c):
            return c
    return ""


def deep_merge(base, override):
    """Recursively merge override dictionary into base dictionary."""
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def list_scenarios():
    """Print all available maps and scenarios."""
    scenarios_dir = find_scenarios_dir()
    if not scenarios_dir or not os.path.isdir(scenarios_dir):
        print("Scenarios directory not found.")
        return

    print("==================================================================")
    print(" VisionPilot CARLA Simulation: Maps & Scenarios")
    print(f" Root: {scenarios_dir}")
    print("==================================================================")

    maps = sorted([d for d in os.listdir(scenarios_dir) if os.path.isdir(os.path.join(scenarios_dir, d))])
    if not maps:
        print(" No map scenario directories found.")
        return

    for m in maps:
        map_dir = os.path.join(scenarios_dir, m)
        print(f"\n Map: \033[1;36m{m}\033[0m")
        scen_files = sorted(glob.glob(os.path.join(map_dir, "*.json")))
        for sf in scen_files:
            fname = os.path.basename(sf)
            scen_id = os.path.splitext(fname)[0]
            if scen_id == "base":
                continue
            try:
                with open(sf) as f:
                    data = json.load(f)
                desc = data.get("description", "No description")
            except Exception:
                desc = "(error reading file)"
            print(f"   * \033[1;32m{scen_id:<18}\033[0m: {desc}")

    print("\n------------------------------------------------------------------")
    print(" Launch commands:")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=lead_vehicle_acc")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=cut_in")
    print("   ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=Town04 scenario:=dense_traffic")
    print("==================================================================")


def load_scenario_config(file_arg="", map_arg="", scenario_arg=""):
    """Load and merge configuration according to 1-map-multiple-scenarios architecture."""
    scenarios_dir = find_scenarios_dir()

    # Case 1: An explicit JSON file path is provided via -f
    if file_arg and os.path.isfile(file_arg):
        with open(file_arg) as f:
            cfg = json.load(f)
        # If this JSON is inside a map scenario directory and not base.json itself, inherit base.json
        file_dir = os.path.abspath(os.path.dirname(file_arg))
        base_path = os.path.join(file_dir, "base.json")
        if os.path.basename(file_arg) != "base.json" and os.path.isfile(base_path):
            with open(base_path) as bf:
                base_cfg = json.load(bf)
            cfg = deep_merge(base_cfg, cfg)
        if map_arg:
            cfg["map"] = map_arg
        return cfg

    # Case 2: Shorthand passed via -f (e.g., -f PoV_Highway/lead_vehicle_acc or -f lead_vehicle_acc)
    if file_arg and scenarios_dir:
        target = file_arg[:-5] if file_arg.endswith(".json") else file_arg
        if "/" in target:
            target_map, target_scen = target.split("/", 1)
        else:
            target_map = map_arg or "Town04"
            target_scen = target
        scen_path = os.path.join(scenarios_dir, target_map, f"{target_scen}.json")
        base_path = os.path.join(scenarios_dir, target_map, "base.json")
        if os.path.isfile(scen_path):
            base_cfg = {}
            if os.path.isfile(base_path):
                with open(base_path) as bf:
                    base_cfg = json.load(bf)
            with open(scen_path) as sf:
                scen_cfg = json.load(sf)
            cfg = deep_merge(base_cfg, scen_cfg)
            if map_arg:
                cfg["map"] = map_arg
            return cfg

    # Case 3: Scenario and/or map specified via arguments
    target_map = map_arg if map_arg else "Town04"
    if scenario_arg and scenarios_dir:
        scen_name = scenario_arg[:-5] if scenario_arg.endswith(".json") else scenario_arg
        scen_path = os.path.join(scenarios_dir, target_map, f"{scen_name}.json")
        base_path = os.path.join(scenarios_dir, target_map, "base.json")
        if not os.path.isfile(scen_path):
            raise FileNotFoundError(
                f"Scenario '{scen_name}' not found for map '{target_map}'. "
                f"Looked in: {scen_path}. Run with --list to see available scenarios."
            )
        base_cfg = {}
        if os.path.isfile(base_path):
            with open(base_path) as bf:
                base_cfg = json.load(bf)
        with open(scen_path) as sf:
            scen_cfg = json.load(sf)
        cfg = deep_merge(base_cfg, scen_cfg)
        cfg["map"] = target_map
        return cfg

    # Case 4: Only map specified, pick default scenario for that map
    if map_arg and scenarios_dir:
        base_path = os.path.join(scenarios_dir, target_map, "base.json")
        default_scens = ["free_drive", "free_cruise", "base"]
        for ds in default_scens:
            dsp = os.path.join(scenarios_dir, target_map, f"{ds}.json")
            if os.path.isfile(dsp):
                base_cfg = {}
                if os.path.isfile(base_path) and ds != "base":
                    with open(base_path) as bf:
                        base_cfg = json.load(bf)
                with open(dsp) as sf:
                    scen_cfg = json.load(sf)
                cfg = deep_merge(base_cfg, scen_cfg)
                cfg["map"] = target_map
                return cfg

    # Case 5: Default fallback
    if scenarios_dir:
        t4_base = os.path.join(scenarios_dir, "Town04", "base.json")
        t4_free = os.path.join(scenarios_dir, "Town04", "free_cruise.json")
        if os.path.isfile(t4_free):
            with open(t4_base) as bf:
                base_cfg = json.load(bf)
            with open(t4_free) as sf:
                scen_cfg = json.load(sf)
            return deep_merge(base_cfg, scen_cfg)

    # Legacy config/carla916.json fallback
    legacy_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "config", "carla916.json")
    )
    if os.path.isfile(legacy_path):
        with open(legacy_path) as f:
            return json.load(f)

    raise RuntimeError("Could not resolve any CARLA scenario or configuration file.")


def _check_versions(client):
    client_ver = client.get_client_version()
    server_ver = client.get_server_version()
    if client_ver.split("-")[0] != server_ver.split("-")[0]:
        logging.warning(
            "CARLA PythonAPI %s != server %s — API calls may segfault; stage matching wheel",
            client_ver,
            server_ver,
        )
    else:
        logging.info("CARLA client/server version %s", server_ver)


def _setup_weather(world, weather_config):
    """Set CARLA weather from preset name or custom attribute dictionary."""
    if not weather_config:
        return
    if isinstance(weather_config, str):
        preset = getattr(carla.WeatherParameters, weather_config, None)
        if preset is not None:
            world.set_weather(preset)
            logging.info("Applied weather preset: %s", weather_config)
        else:
            logging.warning("Unknown weather preset '%s'", weather_config)
    elif isinstance(weather_config, dict):
        weather = carla.WeatherParameters()
        for k, v in weather_config.items():
            if hasattr(weather, k):
                setattr(weather, k, float(v))
            else:
                logging.warning("Unknown weather parameter '%s'", k)
        world.set_weather(weather)
        logging.info("Applied custom weather: %s", weather_config)


def _setup_vehicle(world, config):
    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    # Support nested "hero" object as well as legacy top-level keys
    hero_cfg = config.get("hero", {})
    vehicle_type = hero_cfg.get("type") or config.get("type", "vehicle.lincoln.mkz_2020")
    vehicle_id = hero_cfg.get("id") or config.get("id", "hero")

    logging.debug("Spawning vehicle: %s (id: %s)", vehicle_type, vehicle_id)

    candidates = bp_library.filter(vehicle_type)
    if not candidates:
        raise ValueError(f"No blueprint found for vehicle type '{vehicle_type}'")
    bp = candidates[0]
    bp.set_attribute("role_name", vehicle_id)
    bp.set_attribute("ros_name", vehicle_id)

    spawn_points = map_.get_spawn_points()
    for i in range(len(spawn_points)):
        waypt = map_.get_waypoint(spawn_points[i].location)
        logging.debug(
            "Spawn Point %d: road %s lane %s section %s",
            i, waypt.road_id, waypt.lane_id, waypt.section_id
        )

    # Priority: SPAWN_INDEX env > hero.spawn_index > spawn_index > 0
    if "spawn_index" in hero_cfg:
        default_idx = int(hero_cfg["spawn_index"])
    else:
        default_idx = int(config.get("spawn_index", 0))

    idx = int(os.environ.get("SPAWN_INDEX", default_idx))
    if not 0 <= idx < len(spawn_points):
        raise IndexError(
            "SPAWN_INDEX {} out of range: map {} has {} spawn points (valid 0..{})".format(
                idx, map_.name, len(spawn_points), len(spawn_points) - 1
            )
        )
    logging.info(
        "map %s: using spawn index %d (default %d; override with SPAWN_INDEX)",
        map_.name,
        idx,
        default_idx,
    )
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


def _setup_npc_traffic(world, traffic_manager, config, hero_actor=None):
    """Spawn NPC traffic based on scenario definition (fixed indices, random, or relative to hero)."""
    bp_library = world.get_blueprint_library()
    map_ = world.get_map()
    spawn_points = map_.get_spawn_points()

    npc_vehicles = []
    for npc in config.get("npc_vehicles", []):
        count = int(npc.get("count", 1))
        for _ in range(count):
            bp_filter = npc.get("type", "vehicle.*")
            candidates = bp_library.filter(bp_filter)
            if not candidates:
                logging.warning("No blueprint matches '%s', skipping", bp_filter)
                continue
            bp = random.choice(candidates)
            if bp.has_attribute("color"):
                color = random.choice(bp.get_attribute("color").recommended_values)
                bp.set_attribute("color", color)

            spawn_pt = None
            # Relative spawning along lane waypoint
            if npc.get("spawn_relative_to") == "hero" and hero_actor is not None:
                hero_transform = hero_actor.get_transform()
                hero_wp = map_.get_waypoint(
                    hero_transform.location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving,
                )
                if hero_wp is not None:
                    distance_m = float(npc.get("distance_m", 30.0))
                    if distance_m >= 0:
                        next_wps = hero_wp.next(distance_m)
                        target_wp = next_wps[0] if next_wps else hero_wp
                    else:
                        prev_wps = hero_wp.previous(abs(distance_m))
                        target_wp = prev_wps[0] if prev_wps else hero_wp

                    lane = npc.get("lane", "same").lower()
                    if lane == "left":
                        left_wp = target_wp.get_left_lane()
                        if left_wp is not None and left_wp.lane_type == carla.LaneType.Driving:
                            target_wp = left_wp
                        else:
                            logging.warning("Left lane not available at %sm, using current lane", distance_m)
                    elif lane == "right":
                        right_wp = target_wp.get_right_lane()
                        if right_wp is not None and right_wp.lane_type == carla.LaneType.Driving:
                            target_wp = right_wp
                        else:
                            logging.warning("Right lane not available at %sm, using current lane", distance_m)

                    spawn_pt = carla.Transform(
                        carla.Location(
                            x=target_wp.transform.location.x,
                            y=target_wp.transform.location.y,
                            z=target_wp.transform.location.z + 0.5,
                        ),
                        target_wp.transform.rotation,
                    )
            elif npc.get("spawn_index") is not None:
                idx = int(npc.get("spawn_index"))
                if 0 <= idx < len(spawn_points):
                    spawn_pt = spawn_points[idx]
                else:
                    logging.warning("npc spawn_index %d out of range (max %d), skipping", idx, len(spawn_points) - 1)
                    continue
            else:
                # Random spawn point, avoiding hero spawn index if possible
                hero_cfg = config.get("hero", {})
                hero_idx = int(hero_cfg.get("spawn_index", config.get("spawn_index", 0)))
                free = [p for i, p in enumerate(spawn_points) if i != hero_idx]
                if free:
                    spawn_pt = random.choice(free)
                else:
                    logging.warning("No free spawn points for random NPC")
                    continue

            if spawn_pt is None:
                continue

            actor = world.try_spawn_actor(bp, spawn_pt)
            if actor is None:
                logging.warning("Failed to spawn NPC '%s' (collision at spawn point)", bp_filter)
                continue

            if npc.get("autopilot", True):
                actor.set_autopilot(True, traffic_manager.get_port())
                if "speed_limit_percentage" in npc:
                    traffic_manager.vehicle_percentage_speed_difference(
                        actor, float(npc["speed_limit_percentage"])
                    )
                if "keep_lane" in npc and npc["keep_lane"]:
                    traffic_manager.auto_lane_change(actor, False)

            npc_vehicles.append(actor)

    logging.info("Spawned %d NPC vehicles", len(npc_vehicles))
    return npc_vehicles


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
    npc_vehicles = []
    original_settings = None

    try:
        config = load_scenario_config(
            file_arg=args.file,
            map_arg=args.map,
            scenario_arg=args.scenario,
        )

        map_name = args.map if args.map else config.get("map", "Town04")
        scen_name = config.get("scenario_name", "custom")
        desc = config.get("description", "")

        logging.info("Starting Scenario: [%s] on Map [%s]", scen_name, map_name)
        if desc:
            logging.info("Scenario Description: %s", desc)

        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        _check_versions(client)

        current_world_name = client.get_world().get_map().name
        # Only reload world if map differs to avoid unnecessary reload time
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
        settings.fixed_delta_seconds = 0.1  # 10 Hz fixed simulation step
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # Apply weather
        _setup_weather(world, config.get("weather"))

        # Spawn ego vehicle and sensors
        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        # Spawn NPC vehicles
        npc_vehicles = _setup_npc_traffic(world, traffic_manager, config, hero_actor=vehicle)

        world.tick()  # Settle physics before autopilot / spectator setup

        if args.autopilot:
            vehicle.set_autopilot(True)

        spectator = world.get_spectator()

        logging.info("Running... (ego up; telemetry is published by the bridge container)")

        TARGET_HZ = 10.0
        TARGET_PERIOD = 1.0 / TARGET_HZ

        while True:
            loop_start = time.time()

            world.tick()
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
            if original_settings and world:
                logging.info("Restoring original settings")
                try:
                    world.apply_settings(original_settings)
                except Exception as e:
                    logging.debug("Error restoring settings: %s", e)

            for actor in npc_vehicles:
                try:
                    if actor.is_alive:
                        actor.destroy()
                except Exception as e:
                    logging.debug("Error destroying NPC: %s", e)

            for sensor in sensors:
                try:
                    if sensor.is_alive:
                        sensor.destroy()
                except Exception as e:
                    logging.debug("Error destroying sensor: %s", e)

            if vehicle:
                try:
                    if vehicle.is_alive:
                        vehicle.destroy()
                except Exception as e:
                    logging.debug("Error destroying vehicle: %s", e)

        finally:
            signal.signal(signal.SIGINT, signal.default_int_handler)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA Scenario Runner for VisionPilot")
    argparser.add_argument(
        "--host",
        metavar="H",
        default="localhost",
        help="IP of the host CARLA Simulator (default: localhost)",
    )
    argparser.add_argument(
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port of CARLA Simulator (default: 2000)",
    )
    argparser.add_argument(
        "-f", "--file",
        default="",
        help="Direct path to config/scenario JSON or shorthand (e.g. PoV_Highway/lead_vehicle_acc)",
    )
    argparser.add_argument(
        "-m", "--map",
        type=str,
        default="",
        help="CARLA map name (e.g. PoV_Highway, Town04)",
    )
    argparser.add_argument(
        "-s", "--scenario",
        type=str,
        default="",
        help="Scenario name to load (e.g. free_drive, lead_vehicle_acc, cut_in, convoy_traffic, foggy_drive)",
    )
    argparser.add_argument(
        "-l", "--list",
        action="store_true",
        help="List all available maps and scenarios",
    )
    argparser.add_argument(
        "-v", "--verbose",
        action="store_true",
        dest="debug",
        help="Print debug information",
    )
    argparser.add_argument(
        "-a", "--autopilot",
        action="store_true",
        dest="autopilot",
        help="Turn on autopilot for the hero vehicle",
    )

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    if not args.list:
        logging.info("Connecting to CARLA server %s:%s", args.host, args.port)

    main(args)