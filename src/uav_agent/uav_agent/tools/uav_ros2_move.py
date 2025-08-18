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

@tool
def move_drone_ros2(direction: str, distance: float = 5.0) -> str:
    """
    Move the drone using ROS2 offboard commands.

    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param distance: Distance to move in meters (default: 5.0, max: 20.0)
    """
    valid_directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
    if direction not in valid_directions:
        return f"Error: Invalid direction '{direction}'. Valid directions: {', '.join(valid_directions)}"

    if distance <= 0 or distance > 20:
        return "Error: Distance must be between 0 and 20 meters for safety."

    # Calculate velocity components based on direction (NED frame)
    if direction == "forward":
        vx, vy, vz = 2.0, 0.0, 0.0  # 2 m/s forward
    elif direction == "backward":
        vx, vy, vz = -2.0, 0.0, 0.0  # 2 m/s backward
    elif direction == "left":
        vx, vy, vz = 0.0, -2.0, 0.0  # 2 m/s left
    elif direction == "right":
        vx, vy, vz = 0.0, 2.0, 0.0  # 2 m/s right
    elif direction == "up":
        vx, vy, vz = 0.0, 0.0, -1.0  # 1 m/s up (negative Z in NED)
    elif direction == "down":
        vx, vy, vz = 0.0, 0.0, 1.0  # 1 m/s down (positive Z in NED)

    # Calculate movement time
    movement_time = distance / 2.0  # Time to move at 2 m/s

    # Step 1: Switch to offboard mode
    print(f"Switching to offboard mode for {direction} movement...")
    offboard_cmd = """ros2 service call /fmu/in/vehicle_command px4_msgs/srv/VehicleCommand "{
        command: 176,
        param1: 1,
        param2: 6,
        target_system: 1,
        target_component: 1,
        source_system: 1,
        source_component: 1,
        from_external: true
    }" """

    success_mode, stdout_mode, stderr_mode = run_ros2_command(offboard_cmd, timeout=5)

    # Step 2: Send offboard control mode
    offboard_control_cmd = f"""timeout {movement_time + 2}s ros2 topic pub -r 10 /fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode "{{
        position: false,
        velocity: true,
        acceleration: false,
        attitude: false,
        body_rate: false,
        thrust_and_torque: false,
        direct_actuator: false,
        timestamp: 0
    }}" &"""

    success1, stdout1, stderr1 = run_ros2_command(offboard_control_cmd)

    # Step 3: Send velocity setpoint
    velocity_cmd = f"""timeout {movement_time + 2}s ros2 topic pub -r 10 /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{{
        position: [NaN, NaN, NaN],
        velocity: [{vx}, {vy}, {vz}],
        acceleration: [NaN, NaN, NaN],
        jerk: [NaN, NaN, NaN],
        yaw: NaN,
        yawspeed: NaN,
        timestamp: 0
    }}" &"""

    success2, stdout2, stderr2 = run_ros2_command(velocity_cmd)

    if success1 and success2:
        # Wait for movement to complete
        print(f"Moving {direction} for {movement_time:.1f} seconds...")
        time.sleep(movement_time + 1)

        # Stop movement
        stop_result = stop_drone_movement.invoke("")

        return f"Successfully moved {direction} for {distance} meters using ROS2. {stop_result}"
    else:
        return f"Failed to move {direction}: {stderr1 if stderr1 else stderr2}"

@tool
def stop_drone_movement() -> str:
    """Stop drone movement by sending zero velocity and switching back to position mode."""

    # Send zero velocity command
    stop_cmd = """ros2 topic pub --times 5 /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{
        position: [NaN, NaN, NaN],
        velocity: [0.0, 0.0, 0.0],
        acceleration: [NaN, NaN, NaN],
        jerk: [NaN, NaN, NaN],
        yaw: NaN,
        yawspeed: NaN,
        timestamp: 0
    }" """

    success1, stdout1, stderr1 = run_ros2_command(stop_cmd)

    # Switch back to position mode
    position_mode_cmd = """ros2 service call /fmu/in/vehicle_command px4_msgs/srv/VehicleCommand "{
        command: 176,
        param1: 1,
        param2: 3,
        target_system: 1,
        target_component: 1,
        source_system: 1,
        source_component: 1,
        from_external: true
    }" """

    success2, stdout2, stderr2 = run_ros2_command(position_mode_cmd, timeout=3)

    if success1:
        return "Movement stopped and switched to position hold mode."
    else:
        return f"Failed to stop movement: {stderr1}"
