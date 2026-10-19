# VisionPilot ⇄ CARLA Simulation & ROS 2 Bridge

This package connects **VisionPilot** to **CARLA Simulator** for closed-loop ADAS simulation, supporting both **CARLA 0.9.16** and **CARLA on Unreal Engine 5.8 (`ue58-dev`)** with configurable ROS 2 Middleware (RMW).

---

## 1. CARLA UE 5.8 (`ue58-dev`) & RMW Architecture

In the latest CARLA version on Unreal Engine 5.8 (`ue58-dev`), CARLA decoupled its native ROS 2 connector from a hardcoded FastDDS dependency, allowing dynamic selection of the RMW implementation:
- **`fastdds`** (`rmw_fastrtps_cpp`) — Default
- **`cyclonedds`** (`rmw_cyclonedds_cpp`) — Standard for Autoware
- **`zenoh`** (`rmw_zenoh_cpp`) — Low-latency experimental transport

### Configuring Matching RMW
VisionPilot is a native C++ ROS 2 application linked against `rclcpp`. It dynamically uses whatever RMW is defined in `RMW_IMPLEMENTATION`.

To ensure seamless communication between CARLA and VisionPilot:
1. When launching CARLA:
   ```bash
   ./CarlaUnreal.sh --ros2 --rmw=cyclonedds -nosound
   # or with fastdds:
   ./CarlaUnreal.sh --ros2 --rmw=fastdds -nosound
   ```
2. When launching the ROS 2 bridge:
   ```bash
   ros2 launch carla_bridge_bringup carla_bridge.launch.py rmw:=rmw_cyclonedds_cpp map:=PoV_Highway scenario:=lead_vehicle_acc
   ```
3. When running VisionPilot:
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ./VisionPilot
   ```

---

## 2. Python API Scenario Architecture

Scenarios are defined programmatically using the **CARLA Python API** (the recommended CARLA standard) under `Simulation/CARLA/ROS2/scenarios/`.

```
scenarios/
├── scenario_base.py           # CarlaScenario base class with helper utilities
├── PoV_Highway/               # Autoware Simulation WG 15km Highway Map
│   ├── base.py               # Map defaults (ego vehicle, sensors)
│   ├── free_drive.py         # Open highway driving (LKAS test)
│   ├── lead_vehicle_acc.py   # In-lane lead vehicle 45m ahead (ACC test)
│   ├── cut_in.py             # Adjacent vehicle executing an active cut-in
│   ├── convoy_traffic.py     # 3-vehicle highway convoy & overtaking stream
│   └── static_obstacle.py    # Stalled stationary car with hazard lights (AEB / FCW)
└── Town04/                   # CARLA Urban / Highway Loop Map
    ├── base.py               # Base loop configuration
    ├── free_cruise.py        # Empty highway loop cruise
    ├── dense_traffic.py      # Highway loop with 12 ambient & fixed vehicles
    ├── lead_vehicle_acc.py   # Lead vehicle following on Town04 loop
    ├── cut_in.py             # Active cut-in on Town04 loop
    └── foggy_drive.py        # Volumetric fog & rain adverse weather challenge
```

### Why Python API Scenarios?
Unlike static JSON files, Python API scenarios allow:
1. **Precise Relative Placement**: Vehicles are placed dynamically relative to the hero car along road waypoints (solving the issue on maps like `PoV_Highway` that only have 2 fixed spawn points).
2. **Active Runtime Maneuvers**: In `cut_in.py`, the scenario code dynamically calls `traffic_manager.force_lane_change(vehicle, True)` during the simulation loop to physically cut in front of VisionPilot!
3. **Interactive Hazards**: In `static_obstacle.py`, stalled cars turn on their hazard lights via `set_light_state()`.

---

## 3. Map Behavior & Weather Note

### Why Weather is Static on `PoV_Highway`
`PoV_Highway` is a custom imported Unreal Engine map package from the Autoware Simulation WG (`PoV_Highway.umap`).
In CARLA, dynamic weather changes (`world.set_weather()`) only render visually if the level includes CARLA's `BP_Weather` and `BP_Sky` actors in Unreal Engine. Because `PoV_Highway.umap` was exported without `BP_Weather`, lighting on `PoV_Highway` remains fixed.

### Testing Weather & Low-Visibility in Demos
To evaluate VisionPilot under adverse weather (thick fog, rain, low contrast), use **`Town04`**, which natively contains `BP_Weather`:
```bash
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=Town04 scenario:=foggy_drive
```

---

## 4. Quick Start

### Step 1: Build the ROS 2 Workspace
```bash
cd /home/tranhuunhathuy/Documents/Autoware/vision_pilot/Simulation/CARLA/ROS2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Step 2: List Available Scenarios
```bash
python3 src/carla_bridge_bringup/scripts/config_carla.py --list
```

### Step 3: Start CARLA Simulator
```bash
# CARLA 0.9.16 or UE 5.8:
/path/to/CARLA/CarlaUE4.sh --ros2 -nosound
```

### Step 4: Launch Desired Scenario
```bash
# PoV_Highway: Lead Vehicle ACC Following (45m ahead)
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=lead_vehicle_acc

# PoV_Highway: Active Cut-In Maneuver
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=cut_in

# PoV_Highway: Stalled Vehicle Hazard (AEB / Collision Warning)
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=static_obstacle

# PoV_Highway: 3-Vehicle Traffic Convoy
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=PoV_Highway scenario:=convoy_traffic

# Town04: Adverse Weather / Dense Fog Challenge
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=Town04 scenario:=foggy_drive

# Town04: Dense Ambient Traffic
ros2 launch carla_bridge_bringup carla_bridge.launch.py map:=Town04 scenario:=dense_traffic
```

### Step 5: Run VisionPilot
```bash
cd /home/tranhuunhathuy/Documents/Autoware/vision_pilot/VisionPilot/build
./VisionPilot
```

---

## 5. Authoring a New Scenario (Python API)

To add a new scenario for any map:
1. Create a file `Simulation/CARLA/ROS2/scenarios/<MapName>/<my_scenario>.py`:
```python
import carla
from scenarios.PoV_Highway.base import PoVHighwayBase

class Scenario(PoVHighwayBase):
    scenario_name = "my_scenario"
    description = "Custom test scenario"

    def setup_npcs(self):
        # Place a lead vehicle 40m ahead in the same lane
        wp = self.get_waypoint_ahead(distance_m=40.0, lane="same")
        self.spawn_npc("vehicle.tesla.model3", wp.transform, autopilot=True, speed_pct=20.0)

    def tick(self):
        # Optional: per-tick logic (triggers, speed adjustments)
        pass
```
2. Re-run `colcon build` to update the package data files.
3. Launch with `scenario:=my_scenario`!
