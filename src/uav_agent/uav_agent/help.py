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


def get_help(examples):
    """Generate help text for the UAV agent."""
    help_text = """
# ROSA UAV Agent Help

## Overview
The ROSA UAV Agent enables natural language control of PX4-based drones through the ROSA framework.
You can control the drone using conversational commands while the system ensures safety protocols.

## Safety First
- Always follow the proper sequence: Connect → Arm → Takeoff → Fly → Land → Disarm
- The system will prevent unsafe operations and guide you through proper procedures
- Emergency stop is available but should only be used in true emergencies

## Basic Commands
- "Connect to the drone"
- "Arm the drone"
- "Take off"
- "Fly forward for 2 seconds"
- "Move left for 1 second"
- "Land the drone"
- "Disarm the drone"
- "Check drone status"

## Advanced Commands
- "Set manual control with roll 0.3, pitch 0.2, throttle 0.6, yaw 0"
- "Perform a square flight pattern"
- "Hover in place"
- "Emergency stop"

## Available Examples
Try typing 'examples' to see pre-configured flight scenarios, or use any of these commands:
"""
    
    for i, example in enumerate(examples, 1):
        help_text += f"{i}. {example}\n"
    
    help_text += """
## Special Commands
- `help` - Show this help message
- `examples` - Show example commands
- `clear` - Clear chat history
- `exit` - Exit the application

## Safety Notes
- All movements are limited to 10 seconds maximum duration
- The drone will return to hover state after each movement
- Status checks are performed before each operation
- The system prevents operations in unsafe states

## Troubleshooting
- If commands fail, check drone status first
- Ensure Micro-XRCE-DDS-Agent is running
- Verify PX4 SITL or hardware connection
- Use emergency stop only if necessary

For more information, visit: https://github.com/nasa-jpl/rosa
"""
    
    return help_text
