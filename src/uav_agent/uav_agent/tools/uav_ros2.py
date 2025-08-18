#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import subprocess
import time
from langchain.agents import tool

# Global variables for drone state tracking
drone_armed = False
drone_in_air = False

def run_ros2_command(command, timeout=10):
    """Run a ROS2 command and return the result"""
    try:
        # Use bash explicitly and source ROS2 setup
        full_command = f"bash -c 'source /opt/ros/humble/setup.bash && {command}'"
        result = subprocess.run(
            full_command,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout,
            env={'ROS_DOMAIN_ID': '0'}
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timeout"
    except Exception as e:
        return False, "", str(e)

def publish_vehicle_command(command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    """Publish a vehicle command via ROS2"""
    cmd = f"""ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{{
        command: {command},
        param1: {param1},
        param2: {param2},
        param3: {param3},
        param4: {param4},
        param5: {param5},
        param6: {param6},
        param7: {param7},
        target_system: 1,
        target_component: 1,
        source_system: 1,
        source_component: 1,
        from_external: true,
        timestamp: 0
    }}\""""
    return run_ros2_command(cmd)

@tool
def connect_drone() -> str:
    """Check if the drone is connected via ROS2 topics."""
    success, stdout, stderr = run_ros2_command(
        "ros2 topic list | grep -q '/fmu/out/vehicle_status'"
    )
    if success:
        return "Drone is connected and ROS2 topics are available."
    else:
        return "Drone connection not detected. Please ensure PX4 and Micro-XRCE-DDS-Agent are running."

@tool
def arm_drone() -> str:
    """Arm the drone using ROS2 vehicle command."""
    global drone_armed
    
    if drone_armed:
        return "Drone is already armed."
    
    # MAV_CMD_COMPONENT_ARM_DISARM = 400, param1=1 for arm
    success, stdout, stderr = publish_vehicle_command(400, param1=1)
    
    if success:
        drone_armed = True
        return "Arm command sent successfully. Drone should be armed."
    else:
        return f"Failed to send arm command: {stderr}"

@tool
def disarm_drone() -> str:
    """Disarm the drone using ROS2 vehicle command."""
    global drone_armed
    
    if not drone_armed:
        return "Drone is already disarmed."
    
    # MAV_CMD_COMPONENT_ARM_DISARM = 400, param1=0 for disarm
    success, stdout, stderr = publish_vehicle_command(400, param1=0)
    
    if success:
        drone_armed = False
        return "Disarm command sent successfully. Drone should be disarmed."
    else:
        return f"Failed to send disarm command: {stderr}"

@tool
def takeoff_drone() -> str:
    """Make the drone takeoff using ROS2 vehicle command."""
    global drone_armed, drone_in_air
    
    if not drone_armed:
        return "Error: Drone must be armed before takeoff. Please arm the drone first."
    
    if drone_in_air:
        return "Drone is already in the air."
    
    # MAV_CMD_NAV_TAKEOFF = 22, param7 = altitude (5 meters)
    success, stdout, stderr = publish_vehicle_command(22, param7=5.0)
    
    if success:
        drone_in_air = True
        return "Takeoff command sent successfully. Drone should be taking off to 5 meters altitude."
    else:
        return f"Failed to send takeoff command: {stderr}"

@tool
def land_drone() -> str:
    """Land the drone using ROS2 vehicle command."""
    global drone_in_air
    
    if not drone_in_air:
        return "Drone is already on the ground."
    
    # MAV_CMD_NAV_LAND = 21
    success, stdout, stderr = publish_vehicle_command(21)
    
    if success:
        drone_in_air = False
        return "Land command sent successfully. Drone should be landing."
    else:
        return f"Failed to send land command: {stderr}"

@tool
def get_drone_status() -> str:
    """Get current status of the drone via ROS2 topics."""
    global drone_armed, drone_in_air
    
    # Check if topics are available
    success, stdout, stderr = run_ros2_command(
        "ros2 topic list | grep -c '/fmu/out'"
    )
    
    if success and stdout.strip():
        topic_count = int(stdout.strip())
        connected = topic_count > 0
    else:
        connected = False
    
    status = {
        "connected": connected,
        "armed": drone_armed,
        "in_air": drone_in_air,
        "ros2_topics": topic_count if connected else 0
    }
    
    return f"Drone Status: Connected={status['connected']}, Armed={status['armed']}, In Air={status['in_air']}, ROS2 Topics={status['ros2_topics']}"

@tool
def emergency_stop() -> str:
    """Emergency stop - immediately disarm the drone."""
    global drone_armed, drone_in_air
    
    # MAV_CMD_COMPONENT_ARM_DISARM = 400, param1=0 for disarm, param2=21196 for force
    success, stdout, stderr = publish_vehicle_command(400, param1=0, param2=21196)
    
    if success:
        drone_armed = False
        drone_in_air = False
        return "EMERGENCY STOP: Force disarm command sent!"
    else:
        return f"Failed to send emergency stop command: {stderr}"

@tool
def set_flight_mode(mode: str) -> str:
    """Set flight mode using ROS2 vehicle command."""
    mode_map = {
        "manual": 1,
        "altitude": 2,
        "position": 3,
        "auto": 4,
        "acro": 5,
        "offboard": 6,
        "stabilized": 7,
        "rattitude": 8,
        "takeoff": 9,
        "land": 10,
        "follow_me": 11,
        "return": 12,
        "hold": 13
    }
    
    if mode.lower() not in mode_map:
        return f"Unknown flight mode: {mode}. Available modes: {', '.join(mode_map.keys())}"
    
    # MAV_CMD_DO_SET_MODE = 176
    mode_id = mode_map[mode.lower()]
    success, stdout, stderr = publish_vehicle_command(176, param1=1, param2=mode_id)
    
    if success:
        return f"Flight mode change to {mode} sent successfully."
    else:
        return f"Failed to set flight mode: {stderr}"
