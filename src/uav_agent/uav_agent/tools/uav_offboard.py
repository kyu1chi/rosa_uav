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
import tempfile
import os
from langchain.agents import tool

def execute_mavsdk_script(script_content):
    """Execute a MAVSDK script in a separate process"""
    try:
        # Create a temporary script file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(script_content)
            script_path = f.name
        
        # Execute the script
        result = subprocess.run(
            ['python', script_path],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        # Clean up
        os.unlink(script_path)
        
        return result.returncode == 0, result.stdout, result.stderr
    except Exception as e:
        return False, "", str(e)

@tool
def move_drone_offboard(direction: str, distance: float = 5.0) -> str:
    """
    Move the drone using MAVSDK offboard mode.
    
    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param distance: Distance to move in meters (default: 5.0, max: 20.0)
    """
    valid_directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
    if direction not in valid_directions:
        return f"Error: Invalid direction '{direction}'. Valid directions: {', '.join(valid_directions)}"
    
    if distance <= 0 or distance > 20:
        return "Error: Distance must be between 0 and 20 meters for safety."
    
    script = f'''
import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def move_drone_offboard():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Connected to drone")
                break
        
        # Check if drone is in air
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("Error: Drone is not in air")
                return
            print("Confirmed: Drone is in air")
            break
        
        # Calculate velocity components (body frame)
        if "{direction}" == "forward":
            vx, vy, vz = 2.0, 0.0, 0.0  # 2 m/s forward
        elif "{direction}" == "backward":
            vx, vy, vz = -2.0, 0.0, 0.0  # 2 m/s backward
        elif "{direction}" == "left":
            vx, vy, vz = 0.0, -2.0, 0.0  # 2 m/s left
        elif "{direction}" == "right":
            vx, vy, vz = 0.0, 2.0, 0.0  # 2 m/s right
        elif "{direction}" == "up":
            vx, vy, vz = 0.0, 0.0, -2.0  # 2 m/s up
        elif "{direction}" == "down":
            vx, vy, vz = 0.0, 0.0, 2.0  # 2 m/s down
        
        # Calculate movement time
        movement_time = {distance} / 2.0  # Time to move at 2 m/s
        
        print(f"Starting offboard mode for {{movement_time}} seconds...")
        
        # Set initial setpoint before starting offboard mode
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        
        # Start offboard mode
        await drone.offboard.start()
        print("Offboard mode started")
        
        # Move in the specified direction
        print(f"Moving {{direction}} at {{vx}}, {{vy}}, {{vz}} m/s for {{movement_time}} seconds")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, 0.0)
        )
        
        # Wait for movement to complete
        await asyncio.sleep(movement_time)
        
        # Stop movement
        print("Stopping movement...")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        
        # Wait a bit in hover
        await asyncio.sleep(1.0)
        
        # Stop offboard mode
        await drone.offboard.stop()
        print("Offboard mode stopped")
        
        print(f"Movement completed: {{direction}} for {{distance}} meters")
        
    except Exception as e:
        print(f"Error: {{e}}")
        try:
            await drone.offboard.stop()
        except:
            pass

asyncio.run(move_drone_offboard())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Movement completed" in stdout:
        return f"Successfully moved {direction} for {distance} meters using MAVSDK offboard mode."
    else:
        return f"Failed to move {direction}: {stderr if stderr else stdout}"

@tool
def hover_drone() -> str:
    """Make the drone hover in place using offboard mode."""
    
    script = '''
import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def hover_drone():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        
        # Set hover setpoint
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        
        # Start offboard mode for hovering
        await drone.offboard.start()
        print("Hovering in offboard mode")
        
        # Hover for 2 seconds
        await asyncio.sleep(2.0)
        
        # Stop offboard mode
        await drone.offboard.stop()
        print("Hover completed")
        
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(hover_drone())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Hover completed" in stdout:
        return "Drone is now hovering in place."
    else:
        return f"Failed to hover: {stderr if stderr else stdout}"
