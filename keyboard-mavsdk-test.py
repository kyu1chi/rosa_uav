import asyncio
import random
from mavsdk import System
import KeyPressModule as kp
from langchain.agents import tool
from langchain_openai import ChatOpenAI
from rosa import ROSA
import threading
import queue

# 无人机控制工具
@tool
def arm_drone():
    """Arm the drone for takeoff."""
    global drone_command_queue
    drone_command_queue.put(("arm", None))
    return "Drone armed successfully"

@tool
def disarm_drone():
    """Disarm the drone."""
    global drone_command_queue
    drone_command_queue.put(("disarm", None))
    return "Drone disarmed successfully"

@tool
def takeoff_drone():
    """Make the drone takeoff."""
    global drone_command_queue
    drone_command_queue.put(("takeoff", None))
    return "Drone taking off"

@tool
def land_drone():
    """Land the drone."""
    global drone_command_queue
    drone_command_queue.put(("land", None))
    return "Drone landing"

@tool
def move_drone(direction: str, duration: float = 1.0):
    """
    Move the drone in a specific direction.
    
    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param duration: Duration to move in seconds
    """
    global drone_command_queue
    drone_command_queue.put(("move", {"direction": direction, "duration": duration}))
    return f"Moving drone {direction} for {duration} seconds"

@tool
def set_manual_control(roll: float, pitch: float, throttle: float, yaw: float):
    """
    Set manual control inputs for the drone.
    
    :param roll: Roll input (-1 to 1)
    :param pitch: Pitch input (-1 to 1) 
    :param throttle: Throttle input (0 to 1)
    :param yaw: Yaw input (-1 to 1)
    """
    global drone_command_queue
    drone_command_queue.put(("manual_control", {"roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw}))
    return f"Manual control set: roll={roll}, pitch={pitch}, throttle={throttle}, yaw={yaw}"

# 全局变量
kp.init()
drone = System()
drone_command_queue = queue.Queue()
roll, pitch, throttle, yaw = 0, 0, 0.5, 0

# ROSA代理初始化
def init_rosa_agent():
    """初始化ROSA代理"""
    # 配置LLM (你需要根据实际情况配置)
    llm = ChatOpenAI(
        model="gpt-3.5-turbo",
        temperature=0.1,
        # 添加你的API密钥配置
    )
    
    # 创建ROSA代理
    agent = ROSA(
        ros_version=2,  # 使用ROS2
        llm=llm,
        tools=[arm_drone, disarm_drone, takeoff_drone, land_drone, move_drone, set_manual_control],
        verbose=True,
        streaming=True
    )
    
    return agent

async def process_drone_commands():
    """处理无人机命令队列"""
    global roll, pitch, throttle, yaw
    
    while True:
        try:
            if not drone_command_queue.empty():
                command, params = drone_command_queue.get_nowait()
                
                if command == "arm":
                    await drone.action.arm()
                elif command == "disarm":
                    await drone.action.disarm()
                elif command == "takeoff":
                    await drone.action.takeoff()
                elif command == "land":
                    await drone.action.land()
                elif command == "move":
                    direction = params["direction"]
                    duration = params["duration"]
                    
                    # 根据方向设置控制输入
                    if direction == "forward":
                        roll, pitch, throttle, yaw = 0.5, 0, 0.6, 0
                    elif direction == "backward":
                        roll, pitch, throttle, yaw = -0.5, 0, 0.6, 0
                    elif direction == "left":
                        roll, pitch, throttle, yaw = 0, -0.5, 0.6, 0
                    elif direction == "right":
                        roll, pitch, throttle, yaw = 0, 0.5, 0.6, 0
                    elif direction == "up":
                        roll, pitch, throttle, yaw = 0, 0, 0.8, 0
                    elif direction == "down":
                        roll, pitch, throttle, yaw = 0, 0, 0.3, 0
                    
                    # 执行移动
                    await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                    await asyncio.sleep(duration)
                    
                    # 恢复悬停
                    roll, pitch, throttle, yaw = 0, 0, 0.5, 0
                    
                elif command == "manual_control":
                    roll = params["roll"]
                    pitch = params["pitch"] 
                    throttle = params["throttle"]
                    yaw = params["yaw"]
                    
        except queue.Empty:
            pass
        except Exception as e:
            print(f"Error processing command: {e}")
            
        await asyncio.sleep(0.1)

async def rosa_chat_interface():
    """ROSA聊天界面"""
    agent = init_rosa_agent()
    
    print("\n=== ROSA无人机控制代理已启动 ===")
    print("你可以使用自然语言控制无人机，例如:")
    print("- '起飞无人机'")
    print("- '向前飞行3秒'") 
    print("- '降落'")
    print("- '解锁无人机'")
    print("输入 'exit' 退出\n")
    
    while True:
        try:
            user_input = input("ROSA> ")
            if user_input.lower() == 'exit':
                break
                
            # 使用ROSA处理用户输入
            response = agent.invoke(user_input)
            print(f"ROSA: {response}\n")
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")

async def getKeyboardInput(my_drone):
    """保持原有的键盘控制功能"""
    global roll, pitch, throttle, yaw
    while True:
        roll, pitch, throttle, yaw = 0, 0, 0.5, 0
        value = 0.5
        if kp.getKey("LEFT"):
            pitch = -value
        elif kp.getKey("RIGHT"):
            pitch = value
        if kp.getKey("UP"):
            roll = value
        elif kp.getKey("DOWN"):
            roll = -value
        if kp.getKey("w"):
            throttle = 1
        elif kp.getKey("s"):
            throttle = 0
        if kp.getKey("a"):
            yaw = -value
        elif kp.getKey("d"):
            yaw = value
        elif kp.getKey("i"):
            asyncio.ensure_future(print_flight_mode(my_drone))
        elif kp.getKey("r"):
            await my_drone.action.arm()
        elif kp.getKey("l"):
            await my_drone.action.land()
        await asyncio.sleep(0.1)

async def print_flight_mode(my_drone):
    async for flight_mode in my_drone.telemetry.flight_mode():
        print("FlightMode:", flight_mode)
        break

async def manual_control_drone(my_drone):
    global roll, pitch, throttle, yaw
    while True:
        print(roll, pitch, throttle, yaw)
        await my_drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
        await asyncio.sleep(0.1)

async def run_drone():
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

async def run():
    """主运行函数"""
    # 启动无人机连接和控制
    asyncio.create_task(run_drone())
    asyncio.create_task(process_drone_commands())
    asyncio.create_task(manual_control_drone(drone))
    asyncio.create_task(getKeyboardInput(drone))
    
    # 启动ROSA聊天界面
    await rosa_chat_interface()

if __name__ == "__main__":
    asyncio.run(run())
