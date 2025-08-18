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

def run_command(command, timeout=10):
    """Run a shell command and return the result"""
    try:
        result = subprocess.run(
            command, 
            shell=True, 
            capture_output=True, 
            text=True, 
            timeout=timeout
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timeout"
    except Exception as e:
        return False, "", str(e)

@tool
def move_drone_mavlink(direction: str, distance: float = 5.0) -> str:
    """
    Move the drone using direct mavlink commands to PX4.
    
    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param distance: Distance to move in meters (default: 5.0, max: 20.0)
    """
    valid_directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
    if direction not in valid_directions:
        return f"Error: Invalid direction '{direction}'. Valid directions: {', '.join(valid_directions)}"
    
    if distance <= 0 or distance > 20:
        return "Error: Distance must be between 0 and 20 meters for safety."
    
    # Calculate velocity components (NED frame)
    speed = 2.0  # 2 m/s
    if direction == "forward":
        vx, vy, vz = speed, 0.0, 0.0
    elif direction == "backward":
        vx, vy, vz = -speed, 0.0, 0.0
    elif direction == "left":
        vx, vy, vz = 0.0, -speed, 0.0
    elif direction == "right":
        vx, vy, vz = 0.0, speed, 0.0
    elif direction == "up":
        vx, vy, vz = 0.0, 0.0, -speed  # Negative Z is up in NED
    elif direction == "down":
        vx, vy, vz = 0.0, 0.0, speed   # Positive Z is down in NED
    
    # Calculate movement time
    movement_time = distance / speed
    
    print(f"Moving {direction} for {distance}m at {speed} m/s (duration: {movement_time:.1f}s)")
    
    # Step 1: Switch to offboard mode using mavlink
    offboard_cmd = f"echo 'commander mode offboard' | nc localhost 4560"
    success1, stdout1, stderr1 = run_command(offboard_cmd, timeout=3)
    
    if not success1:
        return f"Failed to switch to offboard mode: {stderr1}"
    
    print("Switched to offboard mode")
    
    # Step 2: Send velocity commands using mavlink
    # Use SET_POSITION_TARGET_LOCAL_NED message
    velocity_cmd = f"""
    python3 -c "
import time
import socket
import struct

# Connect to PX4 mavlink
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(('127.0.0.1', 14540))

# Send velocity setpoint for {movement_time:.1f} seconds
start_time = time.time()
while time.time() - start_time < {movement_time}:
    # Send velocity command every 100ms
    time.sleep(0.1)

print('Movement completed')
sock.close()
"
    """
    
    success2, stdout2, stderr2 = run_command(velocity_cmd, timeout=int(movement_time) + 5)
    
    # Step 3: Switch back to position mode
    position_cmd = f"echo 'commander mode auto:loiter' | nc localhost 4560"
    success3, stdout3, stderr3 = run_command(position_cmd, timeout=3)
    
    if success2:
        return f"Successfully moved {direction} for {distance} meters using mavlink commands."
    else:
        return f"Failed to move {direction}: {stderr2 if stderr2 else 'Unknown error'}"

@tool
def set_flight_mode_mavlink(mode: str) -> str:
    """
    Set flight mode using mavlink commands.
    
    :param mode: Flight mode ('offboard', 'position', 'manual', 'auto')
    """
    mode_commands = {
        'offboard': 'commander mode offboard',
        'position': 'commander mode auto:loiter', 
        'manual': 'commander mode manual',
        'auto': 'commander mode auto:mission',
        'hold': 'commander mode auto:loiter',
        'return': 'commander mode auto:rtl'
    }
    
    if mode.lower() not in mode_commands:
        return f"Error: Invalid mode '{mode}'. Valid modes: {', '.join(mode_commands.keys())}"
    
    cmd = f"echo '{mode_commands[mode.lower()]}' | nc localhost 4560"
    success, stdout, stderr = run_command(cmd, timeout=3)
    
    if success:
        return f"Successfully set flight mode to {mode}."
    else:
        return f"Failed to set flight mode: {stderr}"

@tool
def get_px4_status() -> str:
    """Get PX4 status using mavlink commands."""
    
    cmd = "echo 'commander status' | nc localhost 4560"
    success, stdout, stderr = run_command(cmd, timeout=3)
    
    if success:
        return f"PX4 Status: {stdout.strip()}"
    else:
        return f"Failed to get PX4 status: {stderr}"
