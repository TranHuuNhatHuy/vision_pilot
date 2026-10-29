"""5-Phase Highway ADAS Demonstration Benchmark Scenario for PoV_Highway.

Implements the complete scenario breakdown from CES 2027 demonstration:
  Phase 1: First long straight line (Hands-Free Cruise / ISA).
  Phase 2: First left curve (ALK / LDW lane centering).
  Phase 3: Second straight line (Ego approaches and follows lead vehicle @ 50 km/h via ACC).
  Phase 4: Second left curve (Standstill vehicle hazard appears in same lane 200m ahead @ 0 km/h).
  Phase 5: Preceding vehicle cuts out suddenly when approaching standstill;
           Ego detects stopped vehicle and brakes safely (FCW / AEB).

All triggers are 100% EVENT-BASED:
  - Road curvature & station geometry triggers (Entering / clearing curves)
  - Inter-actor spatial proximity triggers (Preceding vehicle closing on standstill obstacle)
"""

import os
import math
import logging
from enum import IntEnum
import carla
from scenarios.PoV_Highway.base import PoVHighwayBase


class Phase(IntEnum):
    PHASE_1_STRAIGHT_CRUISE    = 1  # Long straight line (Hands-free lane keeping / ISA)
    PHASE_2_FIRST_CURVE        = 2  # Entering curve 1 (ALK / LDW lane centering)
    PHASE_3_LEAD_APPROACH_ACC  = 3  # Cleared curve 1: spawn lead car 200m ahead @ 50 km/h
    PHASE_4_STANDSTILL_APPEARS = 4  # Entering curve 2: spawn standstill vehicle 200m ahead @ 0 km/h
    PHASE_5_CUTOUT_AND_AEB     = 5  # Lead car approaches obstacle -> sudden cut-out, ego AEB
    PHASE_COMPLETED            = 6  # Ego stopped safely behind obstacle, test finished


class Scenario(PoVHighwayBase):
    scenario_name = "highway_benchmark"
    description = (
        "5-Phase Event-Driven ADAS Benchmark: "
        "1) Straight Cruise, 2) Curve 1 ALK, 3) Lead Vehicle ACC (@ 50 km/h), "
        "4) Standstill Hazard (@ 0 km/h), 5) Sudden Cut-Out & AEB"
    )

    # ── Tunable Scenario Parameters ────────────────────────────────────────────
    # Fast-start mode: If set to True (or SCENARIO_FAST_START=1), teleports ego to 500m
    # before Curve 1 so the entire 5-phase test runs in ~60-90s without a 7km commute.
    FAST_START = bool(int(os.environ.get("SCENARIO_FAST_START", "1")))

    # Lead vehicle parameters (Phase 3)
    LEAD_SPAWN_DIST_M = float(os.environ.get("LEAD_SPAWN_DIST", 200.0))
    LEAD_SPEED_KMH = float(os.environ.get("LEAD_SPEED_KMH", 50.0))
    LEAD_BLUEPRINT = os.environ.get("LEAD_BLUEPRINT", "vehicle.tesla.model3")

    # Standstill vehicle parameters (Phase 4)
    STANDSTILL_SPAWN_DIST_M = float(os.environ.get("STANDSTILL_SPAWN_DIST", 200.0))
    STANDSTILL_BLUEPRINT = os.environ.get("STANDSTILL_BLUEPRINT", "vehicle.audi.a2")

    # Cut-out trigger distance between preceding car and standstill obstacle (Phase 5)
    CUTOUT_TRIGGER_DIST_M = float(os.environ.get("CUTOUT_TRIGGER_DIST", 45.0))

    # Ambient highway traffic (oncoming vehicles on opposite carriageway & adjacent lane traffic)
    ENABLE_AMBIENT_TRAFFIC = bool(int(os.environ.get("SCENARIO_AMBIENT_TRAFFIC", "1")))
    NUM_ONCOMING_VEHICLES = int(os.environ.get("NUM_ONCOMING", "8"))
    NUM_ADJACENT_VEHICLES = int(os.environ.get("NUM_ADJACENT", "5"))

    def __init__(self, world, traffic_manager, hero_vehicle=None):
        super().__init__(world, traffic_manager, hero_vehicle)
        self.phase = Phase.PHASE_1_STRAIGHT_CRUISE
        self.lead_vehicle = None
        self.standstill_vehicle = None
        self.cutout_executed = False
        self.start_loc = None
        self.start_s = 0.0

    def setup_npcs(self):
        """Initialize scenario tracking state and optional fast-start repositioning."""
        if not self.hero_vehicle or not self.hero_vehicle.is_alive:
            logging.error("[Scenario] Hero vehicle is not valid during setup_npcs")
            return

        # Ensure Traffic Manager does not despawn distant scenario vehicles
        if hasattr(self.traffic_manager, "set_respawn_dormant_vehicles"):
            self.traffic_manager.set_respawn_dormant_vehicles(False)

        hero_wp = self.get_hero_waypoint()
        self.start_loc = self.hero_vehicle.get_location()
        self.start_s = hero_wp.s

        # Optional fast-start: Teleport ego to 500m before the first curve (~s=6600m)
        if self.FAST_START:
            logging.info("[Scenario] SCENARIO_FAST_START active: advancing ego to 500m before Curve 1")
            target_wps = hero_wp.next(6600.0)
            if target_wps:
                target_wp = target_wps[0]
                tf = carla.Transform(
                    carla.Location(
                        target_wp.transform.location.x,
                        target_wp.transform.location.y,
                        target_wp.transform.location.z + 0.5,
                    ),
                    target_wp.transform.rotation,
                )
                self.hero_vehicle.set_transform(tf)
                self.world.tick()
                hero_wp = self.get_hero_waypoint()
                self.start_loc = self.hero_vehicle.get_location()
                self.start_s = hero_wp.s
                logging.info("[Scenario] Hero vehicle repositioned at s=%.1fm for quick demo", self.start_s)

        # Spawn ambient highway traffic (oncoming vehicles and adjacent lane traffic)
        self._spawn_ambient_traffic()

        logging.info("=" * 70)
        logging.info(" [Scenario] 5-Phase Highway Benchmark Initialized")
        logging.info("   Phase 1: Straight Cruise (Hands-free / ISA)")
        logging.info("   Phase 2: First Curve (ALK / LDW)")
        logging.info("   Phase 3: Lead Vehicle Approach 200m @ %0.1f km/h (ACC)", self.LEAD_SPEED_KMH)
        logging.info("   Phase 4: Standstill Hazard 200m @ 0 km/h")
        logging.info("   Phase 5: Preceding Vehicle Sudden Cut-Out (threshold: %0.1fm) -> AEB", self.CUTOUT_TRIGGER_DIST_M)
        logging.info("=" * 70)

    def tick(self):
        """Called every simulation tick. Evaluates event triggers and drives phase state machine."""
        if not self.hero_vehicle or not self.hero_vehicle.is_alive:
            return

        hero_wp = self.get_hero_waypoint()
        hero_loc = self.hero_vehicle.get_location()
        cur_s = hero_wp.s
        curv = self._get_upcoming_curvature(hero_wp, lookahead_m=30.0)

        # ── Phase 1: Straight Cruise -> Curve 1 ─────────────────────────────
        if self.phase == Phase.PHASE_1_STRAIGHT_CRUISE:
            # Event Trigger: Ego enters the curve section of the map
            # Detected when road station reaches curve 1 (~7050m) or curvature rises sharply
            if cur_s >= 7050.0 or (cur_s > 500.0 and curv >= 0.04):
                self._transition_to(
                    Phase.PHASE_2_FIRST_CURVE,
                    f"Ego entered Curve 1 section (s={cur_s:.1f}m, curv={curv:.4f} deg/m)"
                )

        # ── Phase 2: First Curve ALK -> Straight 2 ──────────────────────────
        elif self.phase == Phase.PHASE_2_FIRST_CURVE:
            # Event Trigger: Ego clears the curve section of the map
            # Detected when road station clears curve 1 (~8100m) and road straightens
            if cur_s >= 8100.0 and curv < 0.025:
                self._transition_to(
                    Phase.PHASE_3_LEAD_APPROACH_ACC,
                    f"Ego cleared Curve 1 into Straight 2 (s={cur_s:.1f}m, curv={curv:.4f} deg/m)"
                )
                # Scenario Event: Spawn vehicle NPC 200m ahead of ego @ 50 km/h
                self._spawn_lead_vehicle()

        # ── Phase 3: ACC Approach -> Curve 2 ────────────────────────────────
        elif self.phase == Phase.PHASE_3_LEAD_APPROACH_ACC:
            # Event Trigger: Ego enters the second curve section of the map
            # Detected when road station reaches curve 2 (~9950m) or curvature rises
            if cur_s >= 9950.0 or (cur_s >= 8400.0 and curv >= 0.035):
                self._transition_to(
                    Phase.PHASE_4_STANDSTILL_APPEARS,
                    f"Ego entered Curve 2 section (s={cur_s:.1f}m, curv={curv:.4f} deg/m)"
                )
                # Scenario Event: Spawn standstill vehicle NPC 200m ahead of ego @ 0 km/h
                self._spawn_standstill_vehicle()

        # ── Phase 4: Following through Curve 2 -> Cut-out Proximity Trigger ──
        elif self.phase == Phase.PHASE_4_STANDSTILL_APPEARS:
            # Event Trigger: When preceding vehicle approaches the standstill vehicle
            if (self.lead_vehicle and self.lead_vehicle.is_alive and
                    self.standstill_vehicle and self.standstill_vehicle.is_alive):
                lead_loc = self.lead_vehicle.get_location()
                standstill_loc = self.standstill_vehicle.get_location()
                dist_lead_to_obstacle = lead_loc.distance(standstill_loc)

                if dist_lead_to_obstacle <= self.CUTOUT_TRIGGER_DIST_M:
                    self._transition_to(
                        Phase.PHASE_5_CUTOUT_AND_AEB,
                        f"Preceding vehicle closed to {dist_lead_to_obstacle:.1f}m from standstill obstacle!"
                    )
                    # Scenario Event: Preceding vehicle cuts out suddenly
                    self._execute_cutout()

        # ── Phase 5: Ego AEB / Stopping safely behind obstacle ──────────────
        elif self.phase == Phase.PHASE_5_CUTOUT_AND_AEB:
            if self.standstill_vehicle and self.standstill_vehicle.is_alive:
                dist_ego_to_obstacle = hero_loc.distance(self.standstill_vehicle.get_location())
                ego_speed_kmh = self._get_speed_kmh(self.hero_vehicle)

                # Event: Ego detected obstacle and came to a safe stop
                if ego_speed_kmh < 1.0 and dist_ego_to_obstacle < 35.0:
                    self._transition_to(
                        Phase.PHASE_COMPLETED,
                        f"Ego brought vehicle to a complete safe stop {dist_ego_to_obstacle:.1f}m behind obstacle. BENCHMARK PASS!"
                    )

    # ── Event Action Helpers ───────────────────────────────────────────────────

    def _spawn_ambient_traffic(self):
        """Populate highway with oncoming vehicles on opposite carriageway & adjacent lane traffic."""
        if not self.ENABLE_AMBIENT_TRAFFIC:
            return

        map_ = self.world.get_map()
        hero_wp = self.get_hero_waypoint()
        ego_s = hero_wp.s
        logging.info("[Scenario] Spawning ambient highway traffic...")

        # 1. Adjacent lane traffic on Road 2 (Lane -1, same direction)
        # Staggered in adjacent lane so ego lane (-2) remains clear for the 5-phase test
        adjacent_offsets = [-70.0, 40.0, 110.0, 240.0, 360.0]
        for offset in adjacent_offsets[:self.NUM_ADJACENT_VEHICLES]:
            try:
                adj_s = ego_s + offset
                if 0.0 < adj_s < 14700.0:
                    wp = map_.get_waypoint_xodr(2, -1, adj_s)
                    if wp:
                        actor = self.spawn_npc(
                            "vehicle.*",
                            wp.transform,
                            autopilot=True,
                            speed_pct=random.uniform(-10.0, 15.0),
                            keep_lane=True,
                        )
                        if actor:
                            logging.info("[Scenario] Spawned adjacent lane vehicle at s=%.0fm (%s)", adj_s, actor.type_id)
            except Exception as e:
                logging.debug("Failed spawning adjacent traffic at offset %.1f: %s", offset, e)

        # 2. Oncoming traffic on Road 1 (opposite carriageway, separated by median strip)
        # Stations on Road 1 run parallel to ego's driving corridor
        oncoming_stations = [6800.0, 7200.0, 7600.0, 8000.0, 8400.0, 8800.0, 9200.0, 9600.0]
        for onc_s in oncoming_stations[:self.NUM_ONCOMING_VEHICLES]:
            try:
                wp = map_.get_waypoint_xodr(1, -2, onc_s)
                if wp:
                    actor = self.spawn_npc(
                        "vehicle.*",
                        wp.transform,
                        autopilot=True,
                        speed_pct=random.uniform(-15.0, 10.0),
                        keep_lane=True,
                    )
                    if actor:
                        logging.info("[Scenario] Spawned oncoming vehicle at s=%.0fm (%s)", onc_s, actor.type_id)
            except Exception as e:
                logging.debug("Failed spawning oncoming traffic at s=%.1f: %s", onc_s, e)

    def _spawn_lead_vehicle(self):
        """Spawn preceding vehicle 200m ahead in lane @ 50 km/h."""
        try:
            wp = self.get_waypoint_ahead(distance_m=self.LEAD_SPAWN_DIST_M, lane="same")
            self.lead_vehicle = self.spawn_npc(
                self.LEAD_BLUEPRINT,
                wp.transform,
                autopilot=True,
                speed_pct=0.0,
                keep_lane=True,
            )
            if self.lead_vehicle:
                # In CARLA Traffic Manager on PoV_Highway (default speed ~30 km/h):
                # A negative percentage drives faster: (1.0 - 50.0 / 30.0) * 100 = -66.7% yields 50 km/h.
                pct_diff = (1.0 - (self.LEAD_SPEED_KMH / 30.0)) * 100.0
                self.traffic_manager.vehicle_percentage_speed_difference(self.lead_vehicle, float(pct_diff))
                if hasattr(self.traffic_manager, "set_desired_speed"):
                    self.traffic_manager.set_desired_speed(self.lead_vehicle, float(self.LEAD_SPEED_KMH))

                logging.info(
                    "[Scenario] SPAWNED Preceding Vehicle (%s) 200m ahead @ %0.1f km/h",
                    self.lead_vehicle.type_id, self.LEAD_SPEED_KMH
                )
        except Exception as e:
            logging.error("[Scenario] Failed to spawn lead vehicle: %s", e)

    def _spawn_standstill_vehicle(self):
        """Spawn standstill vehicle 200m ahead in lane @ 0 km/h."""
        try:
            wp = self.get_waypoint_ahead(distance_m=self.STANDSTILL_SPAWN_DIST_M, lane="same")
            self.standstill_vehicle = self.spawn_static_obstacle(
                self.STANDSTILL_BLUEPRINT,
                wp.transform,
            )
            if self.standstill_vehicle:
                # Lock brakes completely so it behaves as an unyielding stationary hazard
                self.standstill_vehicle.apply_control(
                    carla.VehicleControl(hand_brake=True, throttle=0.0, brake=1.0)
                )
                try:
                    self.standstill_vehicle.set_light_state(
                        carla.VehicleLightState(carla.VehicleLightState.Hazard | carla.VehicleLightState.Brake)
                    )
                except Exception:
                    pass

                # Disable collision detection between lead and obstacle in TM so lead doesn't brake prematurely
                if self.lead_vehicle and self.lead_vehicle.is_alive:
                    self.traffic_manager.collision_detection(self.lead_vehicle, self.standstill_vehicle, False)

                logging.info(
                    "[Scenario] SPAWNED Standstill Obstacle (%s) 200m ahead @ 0 km/h",
                    self.standstill_vehicle.type_id
                )
        except Exception as e:
            logging.error("[Scenario] Failed to spawn standstill vehicle: %s", e)

    def _execute_cutout(self):
        """Force preceding vehicle to suddenly change lanes, exposing the stopped obstacle."""
        if not self.lead_vehicle or not self.lead_vehicle.is_alive:
            return

        try:
            # Crucial: Ignore vehicles and set follow distance to 0 so the lead vehicle
            # does not slam brakes behind the standstill car before executing the cut-out
            self.traffic_manager.ignore_vehicles_percentage(self.lead_vehicle, 100.0)
            self.traffic_manager.distance_to_leading_vehicle(self.lead_vehicle, 0.0)
            if self.standstill_vehicle and self.standstill_vehicle.is_alive:
                self.traffic_manager.collision_detection(self.lead_vehicle, self.standstill_vehicle, False)

            # Determine best adjacent lane for cut-out (prefer left lane, fallback right)
            lead_wp = self.world.get_map().get_waypoint(
                self.lead_vehicle.get_location(),
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
            to_right = True
            offset = 3.85
            if lead_wp.get_left_lane() and lead_wp.get_left_lane().lane_type == carla.LaneType.Driving:
                to_right = False  # Change to left lane
                offset = -3.85
            elif lead_wp.get_right_lane() and lead_wp.get_right_lane().lane_type == carla.LaneType.Driving:
                to_right = True   # Change to right lane
                offset = 3.85

            direction_str = "RIGHT" if to_right else "LEFT"
            logging.info("[Scenario] EXECUTING SUDDEN CUT-OUT to %s lane via lateral offset (%.2fm)!", direction_str, offset)
            self.traffic_manager.vehicle_lane_offset(self.lead_vehicle, offset)
            self.traffic_manager.force_lane_change(self.lead_vehicle, to_right)

        except Exception as e:
            logging.error("[Scenario] Failed to execute cut-out maneuver: %s", e)

    def _transition_to(self, new_phase: Phase, reason: str):
        """Log state machine transition and update current phase."""
        old_phase_name = self.phase.name
        self.phase = new_phase
        logging.info("=" * 60)
        logging.info(" [Scenario Transition] %s -> %s", old_phase_name, new_phase.name)
        logging.info("   Reason: %s", reason)
        logging.info("=" * 60)

    def _get_upcoming_curvature(self, wp, lookahead_m=30.0) -> float:
        """Calculate road curvature (deg/m) over lookahead_m."""
        try:
            next_wps = wp.next(lookahead_m)
            if not next_wps:
                return 0.0
            dyaw = (next_wps[0].transform.rotation.yaw - wp.transform.rotation.yaw + 180.0) % 360.0 - 180.0
            return abs(dyaw) / lookahead_m
        except Exception:
            return 0.0

    @staticmethod
    def _get_speed_kmh(actor) -> float:
        """Retrieve scalar speed in km/h from vehicle actor velocity vector."""
        vel = actor.get_velocity()
        return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

