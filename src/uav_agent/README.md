# ROSA UAV Agent

A natural language interface for controlling PX4 drones using the ROSA (Robot Operating System Agent) framework.

## Overview

The ROSA UAV Agent enables intuitive control of PX4-based drones through natural language commands. It integrates with the PX4 autopilot system via Micro-XRCE-DDS-Agent and provides a safe, user-friendly interface for drone operations.

## Features

- **Natural Language Control**: Control drones using conversational commands
- **Safety First**: Built-in safety protocols and command validation
- **Real-time Streaming**: Live response streaming with rich console output
- **Flight Patterns**: Pre-defined flight patterns (square, circle, etc.)
- **Status Monitoring**: Real-time drone status checking
- **Emergency Controls**: Emergency stop functionality

## Prerequisites

1. **PX4 Autopilot**: Either PX4 SITL (Software In The Loop) or actual hardware
2. **Micro-XRCE-DDS-Agent**: For communication between ROS2 and PX4
3. **ROS2**: Humble or later recommended
4. **Python Dependencies**: Installed via ROSA package requirements

## Installation

1. Ensure ROSA is installed in your workspace
2. Build the UAV agent package:
   ```bash
   cd /path/to/your/workspace
   colcon build --packages-select uav_agent
   source install/setup.bash
   ```

## Usage

### Starting the System

1. **Start PX4 SITL** (for simulation):
   ```bash
   cd /path/to/PX4-Autopilot
   make px4_sitl gazebo
   ```

2. **Start Micro-XRCE-DDS-Agent**:
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

3. **Launch the UAV Agent**:
   ```bash
   # Using ROS2 launch
   ros2 launch uav_agent uav_agent.launch.py
   
   # Or directly run the script
   cd src/uav_agent/scripts
   python3 uav_agent.py
   ```

### Basic Commands

- `"Connect to the drone"`
- `"Arm the drone"`
- `"Take off"`
- `"Fly forward for 3 seconds"`
- `"Move left for 2 seconds"`
- `"Land the drone"`
- `"Disarm the drone"`

### Advanced Commands

- `"Execute a square flight pattern"`
- `"Perform a circular flight pattern"`
- `"Set manual control with roll 0.3, pitch 0.2, throttle 0.6, yaw 0"`
- `"Check drone status"`
- `"Emergency stop"`

### Special Commands

- `help` - Show help information
- `examples` - Display example commands
- `clear` - Clear chat history
- `exit` - Exit the application

## Safety Features

- **Sequential Command Execution**: Commands are executed in proper sequence
- **Status Validation**: System checks drone status before each operation
- **Movement Limits**: Maximum 10-second duration for safety
- **Emergency Stop**: Immediate disarm capability
- **Hover Return**: Automatic return to hover state after movements

## Flight Sequence

Always follow this sequence for safe operation:

1. **Connect** → 2. **Arm** → 3. **Takeoff** → 4. **Flight Operations** → 5. **Land** → 6. **Disarm**

## Troubleshooting

### Common Issues

1. **"Drone not connected"**
   - Ensure PX4 SITL or hardware is running
   - Check Micro-XRCE-DDS-Agent is active
   - Verify UDP port 14540 is available

2. **"Command failed"**
   - Check drone status first
   - Ensure proper command sequence
   - Verify drone is in correct state for operation

3. **"LLM connection failed"**
   - Ensure Ollama is running with Llama 3.1 model
   - Check network connectivity
   - Verify model is downloaded: `ollama pull llama3.1`

### Debug Mode

Run with verbose output for debugging:
```bash
ros2 launch uav_agent uav_agent.launch.py verbose:=true
```

## Architecture

- **ROSA Framework**: Core natural language processing
- **MAVSDK**: PX4 communication library
- **Langchain + Ollama**: LLM integration with Llama 3.1
- **Rich Console**: Enhanced terminal interface
- **Async Processing**: Non-blocking command execution

## Safety Notes

⚠️ **IMPORTANT**: This system is designed for educational and research purposes. Always follow local regulations and safety guidelines when operating drones.

- Never operate beyond visual line of sight without proper authorization
- Always have manual override capability
- Test in simulation before using with real hardware
- Ensure adequate space for flight operations
- Follow all applicable aviation regulations

## Contributing

Contributions are welcome! Please follow the ROSA project guidelines and ensure all safety protocols are maintained.

## License

Licensed under the Apache License, Version 2.0. See LICENSE file for details.
