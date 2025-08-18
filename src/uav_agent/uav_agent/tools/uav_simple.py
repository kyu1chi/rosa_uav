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

# Global variables for drone state tracking
drone_armed = False
drone_in_air = False

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
def connect_drone() -> str:
    """Check if the drone is connected."""
    script = '''
import asyncio
from mavsdk import System

async def check_connection():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Connected")
                return
        print("Not connected")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(check_connection())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Connected" in stdout:
        return "Drone is connected and ready for commands."
    else:
        return f"Drone connection failed: {stderr if stderr else stdout}"

@tool
def arm_drone() -> str:
    """Arm the drone for takeoff."""
    global drone_armed
    
    if drone_armed:
        return "Drone is already armed."
    
    script = '''
import asyncio
from mavsdk import System

async def arm_drone():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        
        await drone.action.arm()
        print("Armed successfully")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(arm_drone())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Armed successfully" in stdout:
        drone_armed = True
        return "Drone armed successfully."
    else:
        return f"Failed to arm drone: {stderr if stderr else stdout}"

@tool
def disarm_drone() -> str:
    """Disarm the drone."""
    global drone_armed
    
    if not drone_armed:
        return "Drone is already disarmed."
    
    script = '''
import asyncio
from mavsdk import System

async def disarm_drone():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        
        await drone.action.disarm()
        print("Disarmed successfully")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(disarm_drone())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Disarmed successfully" in stdout:
        drone_armed = False
        return "Drone disarmed successfully."
    else:
        return f"Failed to disarm drone: {stderr if stderr else stdout}"

@tool
def takeoff_drone() -> str:
    """Make the drone takeoff."""
    global drone_armed, drone_in_air
    
    if not drone_armed:
        return "Error: Drone must be armed before takeoff. Please arm the drone first."
    
    if drone_in_air:
        return "Drone is already in the air."
    
    script = '''
import asyncio
from mavsdk import System

async def takeoff_drone():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        
        await drone.action.takeoff()
        print("Takeoff successful")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(takeoff_drone())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Takeoff successful" in stdout:
        drone_in_air = True
        return "Drone takeoff successful."
    else:
        return f"Failed to takeoff: {stderr if stderr else stdout}"

@tool
def land_drone() -> str:
    """Land the drone."""
    global drone_in_air
    
    if not drone_in_air:
        return "Drone is already on the ground."
    
    script = '''
import asyncio
from mavsdk import System

async def land_drone():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        # Check if drone is in air
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("Drone is already on ground")
                return
            break

        print("Sending land command...")
        await drone.action.land()

        # Wait for landing to complete
        print("Waiting for landing to complete...")
        timeout = 30  # 30 seconds timeout
        start_time = asyncio.get_event_loop().time()

        while True:
            async for in_air in drone.telemetry.in_air():
                if not in_air:
                    print("Landing successful - drone is on ground")
                    return
                break

            # Check timeout
            if asyncio.get_event_loop().time() - start_time > timeout:
                print("Landing timeout - but command was sent")
                return

            await asyncio.sleep(1)

    except Exception as e:
        print(f"Error: {e}")

asyncio.run(land_drone())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success and ("Landing successful" in stdout or "Landing timeout" in stdout):
        drone_in_air = False
        if "Landing successful" in stdout:
            return "Drone landing successful - confirmed on ground."
        else:
            return "Land command sent - drone should be landing (timeout waiting for confirmation)."
    else:
        return f"Failed to land: {stderr if stderr else stdout}"

@tool
def return_to_launch() -> str:
    """Return to launch position and land."""
    global drone_in_air

    if not drone_in_air:
        return "Drone is already on the ground."

    script = '''
import asyncio
from mavsdk import System

async def return_to_launch():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        print("Sending return to launch command...")
        await drone.action.return_to_launch()

        # Wait for RTL to complete
        print("Waiting for return to launch to complete...")
        timeout = 45  # 45 seconds timeout for RTL
        start_time = asyncio.get_event_loop().time()

        while True:
            async for in_air in drone.telemetry.in_air():
                if not in_air:
                    print("RTL successful - drone is on ground")
                    return
                break

            # Check timeout
            if asyncio.get_event_loop().time() - start_time > timeout:
                print("RTL timeout - but command was sent")
                return

            await asyncio.sleep(2)

    except Exception as e:
        print(f"Error: {e}")

asyncio.run(return_to_launch())
'''

    success, stdout, stderr = execute_mavsdk_script(script)
    if success and ("RTL successful" in stdout or "RTL timeout" in stdout):
        drone_in_air = False
        if "RTL successful" in stdout:
            return "Return to launch successful - drone is on ground."
        else:
            return "Return to launch command sent - drone should be returning and landing."
    else:
        return f"Failed to return to launch: {stderr if stderr else stdout}"

@tool
def move_drone(direction: str, duration: float = 2.0) -> str:
    """
    Move the drone in a specific direction for a given duration.

    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param duration: Duration to move in seconds (default: 2.0)
    """
    global drone_in_air

    valid_directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
    if direction not in valid_directions:
        return f"Error: Invalid direction '{direction}'. Valid directions: {', '.join(valid_directions)}"

    if duration <= 0 or duration > 10:
        return "Error: Duration must be between 0 and 10 seconds for safety."

    script = f'''
import asyncio
from mavsdk import System

async def move_drone():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        # Check if drone is actually in air
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("Error: Drone is not in air")
                return
            print("Confirmed: Drone is in air")
            break

        # Get current position
        async for position in drone.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            current_alt = position.relative_altitude_m
            break

        # Calculate target position based on direction
        # For 10m movement, use larger offset (approximately 0.0001 degrees = 11 meters)
        offset = 0.0001 * ({duration} / 2.0)  # Scale offset based on duration
        alt_offset = 2.0 * ({duration} / 2.0)  # Scale altitude offset

        if "{direction}" == "forward":
            target_lat = current_lat + offset
            target_lon = current_lon
            target_alt = current_alt
        elif "{direction}" == "backward":
            target_lat = current_lat - offset
            target_lon = current_lon
            target_alt = current_alt
        elif "{direction}" == "left":
            target_lat = current_lat
            target_lon = current_lon - offset
            target_alt = current_alt
        elif "{direction}" == "right":
            target_lat = current_lat
            target_lon = current_lon + offset
            target_alt = current_alt
        elif "{direction}" == "up":
            target_lat = current_lat
            target_lon = current_lon
            target_alt = current_alt + alt_offset
        elif "{direction}" == "down":
            target_lat = current_lat
            target_lon = current_lon
            target_alt = max(1.0, current_alt - alt_offset)  # Don't go below 1m

        print(f"Moving {{direction}} for {{duration}} seconds...")

        # Use goto to move to target position
        await drone.action.goto_location(target_lat, target_lon, target_alt, 0)

        # Wait for the specified duration
        await asyncio.sleep({duration})

        print("Movement completed")

    except Exception as e:
        print(f"Error: {{e}}")

asyncio.run(move_drone())
'''

    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Movement completed" in stdout:
        return f"Successfully moved {direction} for {duration} seconds."
    else:
        return f"Failed to move {direction}: {stderr if stderr else stdout}"

def sync_drone_status():
    """Synchronize drone status with actual telemetry"""
    global drone_armed, drone_in_air

    script = '''
import asyncio
from mavsdk import System

async def get_real_status():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        # Get armed status
        async for armed in drone.telemetry.armed():
            print(f"Armed: {armed}")
            break

        # Get in_air status
        async for in_air in drone.telemetry.in_air():
            print(f"InAir: {in_air}")
            break

    except Exception as e:
        print(f"Error: {e}")

asyncio.run(get_real_status())
'''

    success, stdout, stderr = execute_mavsdk_script(script)
    if success:
        # Parse the output to update global variables
        if "Armed: True" in stdout:
            drone_armed = True
        elif "Armed: False" in stdout:
            drone_armed = False

        if "InAir: True" in stdout:
            drone_in_air = True
        elif "InAir: False" in stdout:
            drone_in_air = False

@tool
def get_drone_status() -> str:
    """Get current status of the drone with real-time telemetry."""
    global drone_armed, drone_in_air

    # Sync with actual drone status
    sync_drone_status()

    status = {
        "armed": drone_armed,
        "in_air": drone_in_air
    }

    return f"Drone Status: Armed={status['armed']}, In Air={status['in_air']} (Real-time)"

@tool
def emergency_stop() -> str:
    """Emergency stop - immediately disarm the drone."""
    global drone_armed, drone_in_air
    
    script = '''
import asyncio
from mavsdk import System

async def emergency_stop():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        
        await drone.action.disarm()
        print("Emergency stop successful")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(emergency_stop())
'''
    
    success, stdout, stderr = execute_mavsdk_script(script)
    if success:
        drone_armed = False
        drone_in_air = False
        return "EMERGENCY STOP: Drone disarmed!"
    else:
        return f"Emergency stop failed: {stderr if stderr else stdout}"

@tool
def move_drone_distance(direction: str, distance: float = 5.0) -> str:
    """
    Move the drone in a specific direction for a given distance in meters.

    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param distance: Distance to move in meters (default: 5.0, max: 20.0)
    """
    global drone_in_air

    # First sync the real status
    sync_drone_status()

    if not drone_in_air:
        return "Error: Drone must be in the air to move. Please takeoff first."

    valid_directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
    if direction not in valid_directions:
        return f"Error: Invalid direction '{direction}'. Valid directions: {', '.join(valid_directions)}"

    if distance <= 0 or distance > 20:
        return "Error: Distance must be between 0 and 20 meters for safety."

    script = f'''
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardPositionNed

async def move_drone_distance():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        # Check if drone is actually in air
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("Error: Drone is not in air")
                return
            print("Confirmed: Drone is in air")
            break

        # Get current position
        async for position in drone.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            current_alt = position.relative_altitude_m
            break

        print(f"Current position: lat={{current_lat}}, lon={{current_lon}}, alt={{current_alt}}")

        # Calculate target position based on direction and distance
        # 1 degree latitude ≈ 111,000 meters
        # 1 degree longitude ≈ 111,000 * cos(latitude) meters
        lat_offset = {distance} / 111000.0
        lon_offset = {distance} / (111000.0 * 0.8)  # Approximate for mid-latitudes

        if "{direction}" == "forward":
            target_lat = current_lat + lat_offset
            target_lon = current_lon
            target_alt = current_alt
        elif "{direction}" == "backward":
            target_lat = current_lat - lat_offset
            target_lon = current_lon
            target_alt = current_alt
        elif "{direction}" == "left":
            target_lat = current_lat
            target_lon = current_lon - lon_offset
            target_alt = current_alt
        elif "{direction}" == "right":
            target_lat = current_lat
            target_lon = current_lon + lon_offset
            target_alt = current_alt
        elif "{direction}" == "up":
            target_lat = current_lat
            target_lon = current_lon
            target_alt = current_alt + {distance}
        elif "{direction}" == "down":
            target_lat = current_lat
            target_lon = current_lon
            target_alt = max(1.0, current_alt - {distance})  # Don't go below 1m

        print(f"Target position: lat={{target_lat}}, lon={{target_lon}}, alt={{target_alt}}")
        print(f"Moving {{direction}} for {{distance}} meters...")

        # Use offboard mode for more reliable movement
        await drone.offboard.set_position_ned(
            OffboardPositionNed(0.0, 0.0, 0.0, 0.0)
        )

        # Start offboard mode
        await drone.offboard.start()

        # Calculate NED coordinates for movement
        if "{direction}" == "forward":
            ned_north = {distance}
            ned_east = 0.0
        elif "{direction}" == "backward":
            ned_north = -{distance}
            ned_east = 0.0
        elif "{direction}" == "left":
            ned_north = 0.0
            ned_east = -{distance}
        elif "{direction}" == "right":
            ned_north = 0.0
            ned_east = {distance}
        elif "{direction}" == "up":
            ned_north = 0.0
            ned_east = 0.0
        elif "{direction}" == "down":
            ned_north = 0.0
            ned_east = 0.0

        ned_down = -{distance} if "{direction}" == "up" else ({distance} if "{direction}" == "down" else 0.0)

        # Move to target position
        await drone.offboard.set_position_ned(
            OffboardPositionNed(ned_north, ned_east, ned_down, 0.0)
        )

        # Wait for movement to complete
        wait_time = max(8, {distance} / 2)  # At least 8 seconds
        await asyncio.sleep(wait_time)

        # Stop offboard mode
        await drone.offboard.stop()

        print("Movement completed")

    except Exception as e:
        print(f"Error: {{e}}")

asyncio.run(move_drone_distance())
'''

    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "Movement completed" in stdout:
        return f"Successfully moved {direction} for {distance} meters."
    else:
        return f"Failed to move {direction}: {stderr if stderr else stdout}"

@tool
def takeoff_drone_to_altitude(altitude: float = 5.0) -> str:
    """
    Make the drone takeoff to a specific altitude.

    :param altitude: Target altitude in meters (default: 5.0, max: 50.0)
    """
    global drone_armed, drone_in_air

    if not drone_armed:
        return "Error: Drone must be armed before takeoff. Please arm the drone first."

    if drone_in_air:
        return "Drone is already in the air."

    if altitude <= 0 or altitude > 50:
        return "Error: Altitude must be between 0 and 50 meters for safety."

    script = f'''
import asyncio
from mavsdk import System

async def takeoff_to_altitude():
    drone = System()
    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        # Set takeoff altitude
        await drone.action.set_takeoff_altitude({altitude})
        print(f"Takeoff altitude set to {altitude} meters")

        # Takeoff
        await drone.action.takeoff()
        print(f"Takeoff to {altitude} meters successful")

        # Wait for takeoff to complete
        await asyncio.sleep(5)

        # Verify altitude
        async for position in drone.telemetry.position():
            current_alt = position.relative_altitude_m
            print(f"Current altitude: {{current_alt}} meters")
            break

    except Exception as e:
        print(f"Error: {{e}}")

asyncio.run(takeoff_to_altitude())
'''

    success, stdout, stderr = execute_mavsdk_script(script)
    if success and "successful" in stdout:
        drone_in_air = True
        return f"Drone takeoff to {altitude} meters successful."
    else:
        return f"Failed to takeoff to {altitude} meters: {stderr if stderr else stdout}"
