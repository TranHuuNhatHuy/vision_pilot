"""Base class and utilities for CARLA Python API scenarios in VisionPilot.

Provides standard interfaces and helper methods for:
  - Spawning the hero vehicle and mounting sensor rigs
  - Waypoint-based relative NPC placement
  - Setting weather and environmental conditions
  - Per-tick scenario orchestration (cut-in triggers, dynamic braking)
  - Safe actor cleanup
"""

import logging
import random
import carla


class CarlaScenario:
    """Base class for all Python API scenarios."""

    map_name = "Town04"
    scenario_name = "base"
    description = ""
    hero_type = "vehicle.lincoln.mkz_2020"
    hero_id = "hero"
    hero_spawn_index = 0
    weather = carla.WeatherParameters.ClearNoon

    # Default sensor rig for VisionPilot
    sensors_config = [
        {
            "type": "sensor.camera.rgb",
            "id": "main_cam",
            "spawn_point": {
                "x": 1.544, "y": -0.0243, "z": 2.116,
                "roll": -0.10, "pitch": 0.11, "yaw": 0.23,
            },
            "attributes": {
                "image_size_x": 1920,
                "image_size_y": 1280,
                "fov": 50,
                "sensor_tick": 0.0,
            },
        },
        {
            "type": "sensor.other.imu",
            "id": "imu",
            "spawn_point": {
                "x": 0.0, "y": 0.0, "z": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
            "attributes": {
                "sensor_tick": 0.1,
            },
        },
    ]

    def __init__(self, world, traffic_manager, hero_vehicle=None):
        self.world = world
        self.traffic_manager = traffic_manager
        self.hero_vehicle = hero_vehicle
        self.npc_actors = []
        self.static_actors = []

    def setup_npcs(self):
        """Override in subclasses to spawn and configure NPCs, hazards, or traffic."""
        pass

    def tick(self):
        """Called every simulation tick. Override for dynamic scenario logic (e.g. cut-in)."""
        pass

    def cleanup(self):
        """Destroy all spawned NPC and obstacle actors."""
        for actor in self.npc_actors:
            try:
                if actor.is_alive:
                    actor.set_autopilot(False)
                    actor.destroy()
            except Exception as e:
                logging.debug("Error destroying NPC actor: %s", e)
        for actor in self.static_actors:
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception as e:
                logging.debug("Error destroying static actor: %s", e)
        self.npc_actors.clear()
        self.static_actors.clear()

    # ── Helper Methods ─────────────────────────────────────────────────────────

    def get_hero_waypoint(self):
        """Get the current driving waypoint of the hero vehicle."""
        if not self.hero_vehicle or not self.hero_vehicle.is_alive:
            raise RuntimeError("Hero vehicle is not active")
        map_ = self.world.get_map()
        return map_.get_waypoint(
            self.hero_vehicle.get_location(),
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )

    def get_waypoint_ahead(self, distance_m=30.0, lane="same"):
        """Calculate a target waypoint relative to the hero vehicle along the road network.
        
        Args:
            distance_m: Longitudinal distance ahead (>0) or behind (<0) in meters.
            lane: Target lane: "same", "left", or "right".
        """
        hero_wp = self.get_hero_waypoint()
        if distance_m >= 0:
            next_wps = hero_wp.next(distance_m)
            target_wp = next_wps[0] if next_wps else hero_wp
        else:
            prev_wps = hero_wp.previous(abs(distance_m))
            target_wp = prev_wps[0] if prev_wps else hero_wp

        lane = lane.lower()
        if lane == "left":
            left_wp = target_wp.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                target_wp = left_wp
            else:
                logging.warning("Left lane not available at %sm, using in-lane", distance_m)
        elif lane == "right":
            right_wp = target_wp.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                target_wp = right_wp
            else:
                logging.warning("Right lane not available at %sm, using in-lane", distance_m)

        return target_wp

    def spawn_npc(self, blueprint_filter, transform, autopilot=True, speed_pct=0.0, keep_lane=True):
        """Spawn an NPC vehicle and configure Traffic Manager behaviors.
        
        Args:
            blueprint_filter: Filter string, e.g. 'vehicle.tesla.model3' or 'vehicle.*'
            transform: carla.Transform where to spawn.
            autopilot: Enable CARLA Traffic Manager autopilot.
            speed_pct: Percentage difference from current speed limit (e.g. 20.0 = 20% slower).
            keep_lane: If True, disables automatic lane changes for this vehicle.
        """
        bp_lib = self.world.get_blueprint_library()
        candidates = bp_lib.filter(blueprint_filter)
        if not candidates:
            logging.warning("No blueprint matching '%s'", blueprint_filter)
            return None
        bp = random.choice(candidates)
        if bp.has_attribute("color"):
            color = random.choice(bp.get_attribute("color").recommended_values)
            bp.set_attribute("color", color)

        # Lift slightly to avoid clipping into road surface
        tf = carla.Transform(
            carla.Location(transform.location.x, transform.location.y, transform.location.z + 0.5),
            transform.rotation,
        )
        actor = self.world.try_spawn_actor(bp, tf)
        if actor is not None:
            if autopilot:
                actor.set_autopilot(True, self.traffic_manager.get_port())
                if speed_pct != 0.0:
                    self.traffic_manager.vehicle_percentage_speed_difference(actor, float(speed_pct))
                if keep_lane:
                    self.traffic_manager.auto_lane_change(actor, False)
            self.npc_actors.append(actor)
            logging.info("Spawned NPC %s at (%s, %s, %s)", bp.id, tf.location.x, tf.location.y, tf.location.z)
        else:
            logging.warning("Failed to spawn NPC '%s' at (%s, %s, %s)", bp.id, tf.location.x, tf.location.y, tf.location.z)
        return actor

    def spawn_static_obstacle(self, blueprint_filter, transform):
        """Spawn a static prop or stationary vehicle as a road obstacle."""
        bp_lib = self.world.get_blueprint_library()
        candidates = bp_lib.filter(blueprint_filter)
        if not candidates:
            logging.warning("No blueprint matching '%s'", blueprint_filter)
            return None
        bp = random.choice(candidates)
        tf = carla.Transform(
            carla.Location(transform.location.x, transform.location.y, transform.location.z + 0.3),
            transform.rotation,
        )
        actor = self.world.try_spawn_actor(bp, tf)
        if actor is not None:
            self.static_actors.append(actor)
            logging.info("Spawned static obstacle %s at (%s, %s, %s)", bp.id, tf.location.x, tf.location.y, tf.location.z)
        return actor
