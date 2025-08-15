Usage Instructions
Build the package:
cd src/drone_agent
colcon build --packages-select drone_agent
source install/setup.bash
Install dependencies:
Configure environment:
cp .env.example .env
# Edit .env with your API keys
Launch the drone agent:
Start PX4 SITL (in another terminal):
make px4_sitl gazebo
Use natural language commands:
"Connect to the drone"
"Arm the drone and take off to 10 meters"
"Move forward 5 meters"
"Rotate 90 degrees clockwise"
"Land the drone"
This implementation provides a complete ROS2-based drone control system using ROSA, following the same architecture as the turtle_agent but adapted for drone operations with MAVSDK integration and enhanced safety features.