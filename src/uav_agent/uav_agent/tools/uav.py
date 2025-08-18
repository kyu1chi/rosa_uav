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

import asyncio
from mavsdk import System
from langchain.agents import tool

# Global variables for drone control
drone = System()
drone_connected = False
drone_armed = False
drone_in_air = False

# Manual control state
manual_control_state = {
    "roll": 0.0,
    "pitch": 0.0,
    "throttle": 0.5,
    "yaw": 0.0
}

async def initialize_drone_connection():
    """Initialize connection to the drone via Micro-XRCE-DDS-Agent"""
    global drone_connected
    try:
        # Use the correct UDP format for MAVSDK
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        print("Waiting for drone to connect...")

        # Wait for connection with timeout
        connection_timeout = 10  # seconds
        start_time = asyncio.get_event_loop().time()

        async for state in drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                drone_connected = True
                break

            # Check for timeout
            if asyncio.get_event_loop().time() - start_time > connection_timeout:
                print("Connection timeout reached")
                return False

        if not drone_connected:
            print("Failed to establish connection")
            return False

        # Wait for global position with timeout
        print("Waiting for global position...")
        gps_timeout = 15  # seconds
        start_time = asyncio.get_event_loop().time()

        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position state is good enough for flying.")
                break

            # Check for timeout
            if asyncio.get_event_loop().time() - start_time > gps_timeout:
                print("GPS timeout reached, but connection is established")
                break

        return True
    except Exception as e:
        print(f"Failed to connect to drone: {e}")
        drone_connected = False
        return False

# Removed old command processing code - now using direct synchronous execution

# Initialize connection
async def initialize_uav_system():
    """Initialize the UAV system"""
    global drone_connected

    # Initialize drone connection
    connection_success = await initialize_drone_connection()
    if not connection_success:
        print("Failed to initialize drone connection")
        return False

    print(f"UAV system initialized. Connection status: {drone_connected}")
    return True

@tool
def connect_drone() -> str:
    """Connect to the PX4 drone via Micro-XRCE-DDS-Agent."""
    global drone_connected
    try:
        if drone_connected:
            return "Drone is already connected and ready for commands."
        else:
            return "Drone connection not established. Please restart the UAV agent to reinitialize connection."
    except Exception as e:
        return f"Failed to check drone connection: {e}"

def execute_drone_command_sync(command):
    """Execute a drone command synchronously using the global drone instance"""
    global drone_armed, drone_in_air, drone_connected, drone

    if not drone_connected:
        return "Error: Drone not connected. Please ensure the UAV system is initialized."

    async def run_command():
        try:
            # Execute the command using the global drone instance
            if command == "arm":
                await drone.action.arm()
                return "Drone armed successfully"
            elif command == "disarm":
                await drone.action.disarm()
                return "Drone disarmed successfully"
            elif command == "takeoff":
                await drone.action.takeoff()
                return "Takeoff completed successfully"
            elif command == "land":
                await drone.action.land()
                return "Landing completed successfully"
            else:
                return f"Unknown command: {command}"
        except Exception as e:
            return f"Command execution failed: {e}"

    try:
        # Use a simpler approach - run in a thread pool
        import concurrent.futures

        def run_in_new_loop():
            # Create a new event loop for this thread
            new_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(new_loop)
            try:
                return new_loop.run_until_complete(run_command())
            finally:
                new_loop.close()

        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(run_in_new_loop)
            result = future.result(timeout=30)

        # Update state based on command
        if "successfully" in result:
            if command == "arm":
                drone_armed = True
            elif command == "disarm":
                drone_armed = False
            elif command == "takeoff":
                drone_in_air = True
            elif command == "land":
                drone_in_air = False

        return result
    except Exception as e:
        return f"Command failed: {e}"

@tool
def arm_drone() -> str:
    """Arm the drone for takeoff. This must be done before takeoff."""
    global drone_connected, drone_armed

    if not drone_connected:
        return "Error: Drone not connected. Please connect first."

    if drone_armed:
        return "Drone is already armed and ready for takeoff."

    return execute_drone_command_sync("arm")

@tool
def disarm_drone() -> str:
    """Disarm the drone. This should be done after landing."""
    global drone_armed

    if not drone_armed:
        return "Drone is already disarmed."

    return execute_drone_command_sync("disarm")

@tool
def takeoff_drone() -> str:
    """Make the drone takeoff to default altitude."""
    global drone_armed, drone_connected, drone_in_air

    if not drone_connected:
        return "Error: Drone not connected. Please connect first."

    if drone_in_air:
        return "Drone is already in the air."

    if not drone_armed:
        return "Error: Drone must be armed before takeoff. Please arm the drone first."

    return execute_drone_command_sync("takeoff")

@tool
def land_drone() -> str:
    """Land the drone at current position."""
    global drone_in_air

    if not drone_in_air:
        return "Drone is already on the ground."

    return execute_drone_command_sync("land")

@tool
def move_drone(direction: str, duration: float = 1.0) -> str:
    """
    Move the drone in a specific direction for a given duration.
    
    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param duration: Duration to move in seconds (default: 1.0)
    """
    global drone_command_queue, drone_in_air
    
    if not drone_in_air:
        return "Error: Drone must be in the air to move. Please takeoff first."
    
    valid_directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
    if direction not in valid_directions:
        return f"Error: Invalid direction '{direction}'. Valid directions: {', '.join(valid_directions)}"
    
    if duration <= 0 or duration > 10:
        return "Error: Duration must be between 0 and 10 seconds for safety."
    
    drone_command_queue.put(("move", {"direction": direction, "duration": duration}))
    return f"Moving drone {direction} for {duration} seconds."

@tool
def set_manual_control(roll: float, pitch: float, throttle: float, yaw: float) -> str:
    """
    Set manual control inputs for the drone.
    
    :param roll: Roll input (-1 to 1, positive = right)
    :param pitch: Pitch input (-1 to 1, positive = forward) 
    :param throttle: Throttle input (0 to 1, 0.5 = hover)
    :param yaw: Yaw input (-1 to 1, positive = clockwise)
    """
    global drone_command_queue, drone_in_air
    
    if not drone_in_air:
        return "Error: Drone must be in the air for manual control. Please takeoff first."
    
    # Validate inputs
    if not (-1 <= roll <= 1) or not (-1 <= pitch <= 1) or not (0 <= throttle <= 1) or not (-1 <= yaw <= 1):
        return "Error: Control inputs out of range. Roll/Pitch/Yaw: -1 to 1, Throttle: 0 to 1"
    
    drone_command_queue.put(("manual_control", {
        "roll": roll, 
        "pitch": pitch, 
        "throttle": throttle, 
        "yaw": yaw
    }))
    return f"Manual control set: roll={roll}, pitch={pitch}, throttle={throttle}, yaw={yaw}"

@tool
def get_drone_status() -> str:
    """Get current status of the drone."""
    global drone_connected, drone_armed, drone_in_air
    
    status = {
        "connected": drone_connected,
        "armed": drone_armed,
        "in_air": drone_in_air,
        "manual_control": manual_control_state
    }
    
    return f"Drone Status: Connected={status['connected']}, Armed={status['armed']}, In Air={status['in_air']}, Manual Control={status['manual_control']}"

@tool
def emergency_stop() -> str:
    """Emergency stop - immediately disarm the drone. Use only in emergencies!"""
    global drone_command_queue
    drone_command_queue.put(("disarm", None))
    return "EMERGENCY STOP: Drone disarmed immediately!"
