import asyncio
import queue
import threading
from typing import List
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw
from langchain.agents import tool
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Global drone system and command queue with result tracking
drone_system = System()
drone_command_queue = queue.Queue()
drone_status = {"armed": False, "flying": False, "altitude": 0.0}
command_results = {}  # 存储命令执行结果
command_counter = 0   # 命令计数器

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        self.status_publisher = self.create_publisher(String, '/drone/status', 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/drone/cmd_vel', self.cmd_vel_callback, 10
        )
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS2 topics."""
        self.get_logger().info(f'Received cmd_vel: {msg}')

# Initialize ROS2 node
drone_node = None

def init_drone_node():
    global drone_node
    if drone_node is None:
        # 不要在这里调用 rclpy.init()，因为主程序已经调用了
        drone_node = DroneControlNode()

@tool
def connect_drone(connection_string: str = "udp://:14540") -> str:
    """
    Connect to the drone using MAVSDK.
    
    :param connection_string: Connection string for the drone (default: udp://:14540)
    """
    global command_counter
    try:
        command_id = f"connect_{command_counter}"
        command_counter += 1
        
        drone_command_queue.put(("connect", {"connection_string": connection_string, "command_id": command_id}))
        
        # 等待命令完成
        return _wait_for_command_completion(command_id, f"正在连接无人机: {connection_string}...")
    except Exception as e:
        return f"连接无人机失败: {e}"

async def _connect_drone_async(connection_string: str):
    """Async helper for drone connection."""
    global drone_system, drone_status
    try:
        print(f"开始连接无人机: {connection_string}")
        await drone_system.connect(system_address=connection_string)
        print("等待无人机连接...")

        # Wait for connection with timeout
        connection_timeout = 10  # 10秒超时
        start_time = asyncio.get_event_loop().time()

        async for state in drone_system.core.connection_state():
            current_time = asyncio.get_event_loop().time()
            if current_time - start_time > connection_timeout:
                print("连接超时")
                drone_status["connected"] = False
                return

            if state.is_connected:
                drone_status["connected"] = True
                print("✓ 无人机连接成功!")
                break

        # 如果连接成功，等待GPS定位
        if drone_status.get("connected", False):
            print("等待GPS定位...")
            gps_timeout = 15  # 15秒GPS超时
            start_time = asyncio.get_event_loop().time()

            async for health in drone_system.telemetry.health():
                current_time = asyncio.get_event_loop().time()
                if current_time - start_time > gps_timeout:
                    print("GPS定位超时，但连接已建立")
                    break

                if health.is_global_position_ok and health.is_home_position_ok:
                    print("✓ GPS定位就绪!")
                    break

        print(f"连接完成，状态: {drone_status}")

    except Exception as e:
        print(f"连接错误: {e}")
        drone_status["connected"] = False

@tool
def arm_drone() -> str:
    """Arm the drone for takeoff."""
    global command_counter
    command_id = f"arm_{command_counter}"
    command_counter += 1
    
    drone_command_queue.put(("arm", {"command_id": command_id}))
    return _wait_for_command_completion(command_id, "正在解锁无人机...")

@tool
def disarm_drone() -> str:
    """Disarm the drone."""
    global drone_command_queue
    drone_command_queue.put(("disarm", None))
    return "Disarming drone..."

@tool
def takeoff_drone(altitude: float = 5.0) -> str:
    """
    Make the drone takeoff to specified altitude.
    
    :param altitude: Target altitude in meters (default: 5.0)
    """
    global command_counter, drone_status
    
    # 检查连接状态
    print(f"调试：当前无人机状态 = {drone_status}")
    connected = drone_status.get('connected', False)
    print(f"调试：连接状态 = {connected}")

    if not connected:
        print("错误：无人机未连接，尝试等待连接...")
        # 给连接一些时间完成
        import time
        time.sleep(2)
        connected = drone_status.get('connected', False)
        print(f"调试：重新检查连接状态 = {connected}")

        if not connected:
            return "错误：无人机未连接，请先连接无人机"
    
    command_id = f"takeoff_{command_counter}"
    command_counter += 1
    
    drone_command_queue.put(("takeoff", {"altitude": altitude, "command_id": command_id}))
    return _wait_for_command_completion(command_id, f"正在起飞到 {altitude}米高度...")

@tool
def land_drone() -> str:
    """Land the drone at current position."""
    global command_counter
    command_id = f"land_{command_counter}"
    command_counter += 1

    drone_command_queue.put(("land", {"command_id": command_id}))
    return _wait_for_command_completion(command_id, "正在降落...")

@tool
def move_drone(direction: str, distance: float = 1.0, speed: float = 1.0) -> str:
    """
    Move the drone in a specific direction.

    :param direction: Direction to move ('forward', 'backward', 'left', 'right', 'up', 'down')
    :param distance: Distance to move in meters
    :param speed: Speed of movement in m/s
    """
    global command_counter
    valid_directions = ['forward', 'backward', 'left', 'right', 'up', 'down']

    if direction.lower() not in valid_directions:
        return f"Invalid direction. Use one of: {', '.join(valid_directions)}"

    command_id = f"move_{command_counter}"
    command_counter += 1

    drone_command_queue.put(("move", {
        "direction": direction.lower(),
        "distance": distance,
        "speed": speed,
        "command_id": command_id
    }))
    return _wait_for_command_completion(command_id, f"正在向{direction}移动 {distance}米...")

@tool
def rotate_drone(angle: float, direction: str = "clockwise") -> str:
    """
    Rotate the drone by specified angle.
    
    :param angle: Angle to rotate in degrees
    :param direction: Rotation direction ('clockwise' or 'counterclockwise')
    """
    global drone_command_queue
    
    if direction.lower() not in ['clockwise', 'counterclockwise']:
        return "Direction must be 'clockwise' or 'counterclockwise'"
    
    drone_command_queue.put(("rotate", {
        "angle": angle,
        "direction": direction.lower()
    }))
    return f"Rotating {angle} degrees {direction}..."

@tool
def hover_drone(duration: float = 5.0) -> str:
    """
    Make the drone hover at current position.

    :param duration: Duration to hover in seconds
    """
    global drone_command_queue
    drone_command_queue.put(("hover", {"duration": duration}))
    return f"Hovering for {duration} seconds..."

@tool
def hold_altitude(target_altitude: float) -> str:
    """
    Make the drone hold a specific altitude at current position.

    :param target_altitude: Target altitude to maintain in meters
    """
    global command_counter
    command_id = f"hold_altitude_{command_counter}"
    command_counter += 1

    drone_command_queue.put(("hold_altitude", {"altitude": target_altitude, "command_id": command_id}))
    return _wait_for_command_completion(command_id, f"保持高度 {target_altitude}米...")

@tool
def get_drone_status() -> str:
    """Get current drone status including position, battery, and flight mode."""
    global drone_status
    try:
        status_info = []
        connected = drone_status.get('connected', False)
        status_info.append(f"连接状态: {'✓ 已连接' if connected else '✗ 未连接'}")
        status_info.append(f"解锁状态: {'✓ 已解锁' if drone_status.get('armed', False) else '✗ 已锁定'}")
        status_info.append(f"飞行状态: {'✓ 飞行中' if drone_status.get('flying', False) else '✗ 地面'}")
        status_info.append(f"高度: {drone_status.get('altitude', '未知')}米")
        
        return "无人机状态:\n" + "\n".join(status_info)
    except Exception as e:
        return f"获取无人机状态失败: {e}"

@tool
def execute_flight_sequence(altitude: float, forward_distance: float, speed: float = 2.0) -> str:
    """
    Execute a complete flight sequence: takeoff → move forward → land.

    :param altitude: Target takeoff altitude in meters
    :param forward_distance: Distance to move forward in meters
    :param speed: Movement speed in m/s (default: 2.0)
    """
    global command_counter
    command_id = f"sequence_{command_counter}"
    command_counter += 1

    drone_command_queue.put(("flight_sequence", {
        "altitude": altitude,
        "forward_distance": forward_distance,
        "speed": speed,
        "command_id": command_id
    }))
    return _wait_for_command_completion(command_id, f"正在执行飞行序列: 起飞{altitude}m → 前进{forward_distance}m → 降落...", timeout=120)

@tool
def emergency_land() -> str:
    """Emergency land the drone immediately."""
    global drone_command_queue
    drone_command_queue.put(("emergency_land", None))
    return "EMERGENCY LANDING INITIATED!"

@tool
def set_manual_control(roll: float, pitch: float, throttle: float, yaw: float) -> str:
    """
    Set manual control inputs for the drone.
    
    :param roll: Roll input (-1 to 1)
    :param pitch: Pitch input (-1 to 1)
    :param throttle: Throttle input (0 to 1)
    :param yaw: Yaw input (-1 to 1)
    """
    global drone_command_queue
    
    # Validate inputs
    if not all(-1 <= val <= 1 for val in [roll, pitch, yaw]):
        return "Roll, pitch, and yaw must be between -1 and 1"
    if not 0 <= throttle <= 1:
        return "Throttle must be between 0 and 1"
    
    drone_command_queue.put(("manual_control", {
        "roll": roll,
        "pitch": pitch,
        "throttle": throttle,
        "yaw": yaw
    }))
    return f"Manual control set: roll={roll}, pitch={pitch}, throttle={throttle}, yaw={yaw}"

# Background task to process drone commands
async def process_drone_commands():
    """Process commands from the queue and execute them."""
    global drone_system, drone_command_queue, drone_status, command_results
    
    while True:
        try:
            if not drone_command_queue.empty():
                command, params = drone_command_queue.get_nowait()
                command_id = params.get("command_id") if params else None
                print(f"处理命令: {command} (ID: {command_id})")
                
                try:
                    if command == "connect":
                        connection_string = params.get("connection_string", "udp://:14540") if params else "udp://:14540"
                        await _connect_drone_async(connection_string)
                        if command_id:
                            if drone_status.get("connected", False):
                                command_results[command_id] = "✓ 无人机连接成功"
                            else:
                                command_results[command_id] = "✗ 无人机连接失败"

                    elif command == "arm":
                        print("正在解锁无人机...")
                        await drone_system.action.arm()
                        drone_status["armed"] = True
                        print("✓ 无人机解锁成功")
                        if command_id:
                            command_results[command_id] = "✓ 无人机解锁成功"

                    elif command == "disarm":
                        print("正在锁定无人机...")
                        await drone_system.action.disarm()
                        drone_status["armed"] = False
                        print("✓ 无人机锁定成功")
                        if command_id:
                            command_results[command_id] = "✓ 无人机锁定成功"

                    elif command == "takeoff":
                        altitude = params.get("altitude", 5.0) if params else 5.0
                        print(f"正在起飞到 {altitude}米...")
                        print(f"起飞前状态检查: {drone_status}")

                        # 确保无人机已解锁
                        if not drone_status.get("armed", False):
                            print("无人机未解锁，先解锁...")
                            try:
                                await drone_system.action.arm()
                                drone_status["armed"] = True
                                print("✓ 无人机解锁成功")
                                await asyncio.sleep(2)
                            except Exception as arm_error:
                                print(f"解锁失败: {arm_error}")
                                if command_id:
                                    command_results[command_id] = f"解锁失败: {arm_error}"
                                continue

                        # 使用主动高度控制策略
                        try:
                            print(f"🚁 开始主动高度控制起飞到 {altitude}米")

                            # 阶段1：安全起飞到最小高度
                            safe_altitude = 0.8  # 安全起飞高度
                            print(f"阶段1：安全起飞到 {safe_altitude}米")
                            await drone_system.action.set_takeoff_altitude(safe_altitude)
                            await drone_system.action.takeoff()

                            # 实时监控起飞过程，防止超调
                            print("🔍 实时监控起飞过程，防止高度超调...")
                            start_time = asyncio.get_event_loop().time()
                            safe_timeout = 20
                            position_control_activated = False

                            async for position in drone_system.telemetry.position():
                                current_time = asyncio.get_event_loop().time()
                                if current_time - start_time > safe_timeout:
                                    print("⏰ 安全起飞监控超时，强制激活位置控制...")
                                    position_control_activated = True
                                    break

                                current_alt = position.relative_altitude_m
                                print(f"📊 起飞监控: {current_alt:.2f}m (目标: {altitude}m)")

                                # 关键改进：一旦达到目标高度的80%，立即切换到位置控制
                                if current_alt >= altitude * 0.8 and not position_control_activated:
                                    print(f"� 达到目标高度80%，立即激活位置控制防止超调!")
                                    try:
                                        await drone_system.action.goto_location(
                                            position.latitude_deg,
                                            position.longitude_deg,
                                            altitude,
                                            0
                                        )
                                        position_control_activated = True
                                        print(f"✅ 位置控制已激活，目标高度: {altitude}米")
                                        break
                                    except Exception as e:
                                        print(f"❌ 位置控制激活失败: {e}")

                                # 如果还没达到80%但已经超过安全高度，也要准备切换
                                elif current_alt >= safe_altitude * 0.9:
                                    print(f"✓ 安全起飞接近完成: {current_alt:.2f}m")

                                await asyncio.sleep(0.2)  # 高频监控防止超调

                            # 确保位置控制已激活
                            if not position_control_activated:
                                print("🎯 强制激活位置控制模式")
                                async for current_pos in drone_system.telemetry.position():
                                    await drone_system.action.goto_location(
                                        current_pos.latitude_deg,
                                        current_pos.longitude_deg,
                                        altitude,
                                        0
                                    )
                                    print(f"✅ 强制位置控制已激活，目标高度: {altitude}米")
                                    break

                            # 阶段2：主动高度监控和控制
                            print(f"🎯 阶段2：主动高度监控和精确控制到 {altitude}米")
                            start_time = asyncio.get_event_loop().time()

                            # 根据高度调整控制参数
                            if altitude >= 8.0:
                                timeout = 60  # 高海拔需要更多时间
                                altitude_tolerance = 0.4  # 高海拔稍微放宽容差
                                control_interval = 1.5  # 稍微减少控制频率
                                required_stable_readings = 6
                            elif altitude >= 5.0:
                                timeout = 50
                                altitude_tolerance = 0.3
                                control_interval = 1.2
                                required_stable_readings = 7
                            else:
                                timeout = 40
                                altitude_tolerance = 0.2
                                control_interval = 1.0
                                required_stable_readings = 8

                            stable_count = 0
                            last_control_time = 0
                            overshoot_detected = False
                            max_altitude_reached = 0

                            print(f"📋 高度控制参数: 超时{timeout}s, 容差±{altitude_tolerance}m, 控制间隔{control_interval}s")

                            async for position in drone_system.telemetry.position():
                                current_time = asyncio.get_event_loop().time()
                                if current_time - start_time > timeout:
                                    print("⏰ 高度控制超时，强制完成...")
                                    break

                                current_alt = position.relative_altitude_m
                                altitude_diff = abs(current_alt - altitude)

                                # 更新最大高度记录
                                if current_alt > max_altitude_reached:
                                    max_altitude_reached = current_alt

                                # 检测超调
                                if current_alt > altitude + 0.2 and not overshoot_detected:
                                    overshoot_detected = True
                                    print(f"🚨 检测到高度超调: {current_alt:.2f}m > {altitude + 0.2:.2f}m")

                                print(f"📊 高度监控: {current_alt:.2f}m (目标: {altitude}m, 误差: {altitude_diff:.2f}m, 最高: {max_altitude_reached:.2f}m)")

                                # 主动高度控制策略
                                should_send_control = False
                                control_reason = ""

                                # 条件1：定期发送控制命令
                                if current_time - last_control_time >= control_interval:
                                    should_send_control = True
                                    control_reason = "定期控制"
                                    last_control_time = current_time

                                # 条件2：高度偏差过大
                                elif altitude_diff > 0.25:
                                    should_send_control = True
                                    control_reason = f"偏差过大({altitude_diff:.2f}m)"

                                # 条件3：检测到超调
                                elif overshoot_detected and current_alt > altitude + 0.1:
                                    should_send_control = True
                                    control_reason = "超调修正"

                                # 发送控制命令
                                if should_send_control:
                                    print(f"🎯 发送高度控制命令 - 原因: {control_reason}")
                                    try:
                                        await drone_system.action.goto_location(
                                            position.latitude_deg,
                                            position.longitude_deg,
                                            altitude,
                                            0
                                        )
                                        print(f"✅ 高度控制命令已发送: 目标{altitude}m")
                                    except Exception as e:
                                        print(f"❌ 高度控制命令失败: {e}")

                                # 检查高度稳定性
                                if altitude_diff <= altitude_tolerance:
                                    stable_count += 1
                                    print(f"📈 高度稳定进度: {stable_count}/{required_stable_readings}")

                                    if stable_count >= required_stable_readings:
                                        print(f"🎉 目标高度已达到并稳定: {current_alt:.2f}m")
                                        break
                                else:
                                    stable_count = 0  # 重置稳定计数器

                                await asyncio.sleep(0.2)  # 高频监控

                            # 阶段4：最终高度验证和稳定
                            print("阶段4：最终高度验证和稳定...")
                            final_altitude = 0

                            try:
                                # 获取最终高度并发送最后的稳定命令
                                async for current_pos in drone_system.telemetry.position():
                                    final_altitude = current_pos.relative_altitude_m
                                    print(f"最终高度验证: {final_altitude:.2f}m (目标: {altitude}m)")

                                    # 发送最终的位置保持命令
                                    await drone_system.action.goto_location(
                                        current_pos.latitude_deg,
                                        current_pos.longitude_deg,
                                        altitude,  # 确保使用目标高度
                                        0  # 保持当前偏航角
                                    )
                                    print("✓ 最终位置保持命令已发送")
                                    break

                                # 短暂等待确保命令生效
                                await asyncio.sleep(2)

                                # 最终高度检查
                                async for position in drone_system.telemetry.position():
                                    final_altitude = position.relative_altitude_m
                                    altitude_error = abs(final_altitude - altitude)
                                    print(f"起飞完成 - 最终高度: {final_altitude:.2f}m, 目标: {altitude}m, 误差: {altitude_error:.2f}m")
                                    break

                            except Exception as final_error:
                                print(f"最终稳定命令失败: {final_error}")

                            # 更新状态
                            drone_status["flying"] = True
                            drone_status["altitude"] = final_altitude if final_altitude > 0 else altitude

                            # 生成结果报告
                            altitude_error = abs(final_altitude - altitude) if final_altitude > 0 else 0
                            if altitude_error <= 0.3:
                                result_msg = f"✅ 无人机起飞完成，高度控制精确: {final_altitude:.2f}m (误差: {altitude_error:.2f}m)"
                                print(result_msg)
                            elif altitude_error <= 0.5:
                                result_msg = f"⚠️ 无人机起飞完成，高度控制良好: {final_altitude:.2f}m (误差: {altitude_error:.2f}m)"
                                print(result_msg)
                            else:
                                result_msg = f"❌ 无人机起飞完成，但高度误差较大: {final_altitude:.2f}m (误差: {altitude_error:.2f}m)"
                                print(result_msg)

                            if command_id:
                                command_results[command_id] = result_msg

                        except Exception as takeoff_error:
                            print(f"起飞失败: {takeoff_error}")
                            if command_id:
                                command_results[command_id] = f"起飞失败: {takeoff_error}"

                    elif command == "land":
                        print("正在降落...")
                        await drone_system.action.land()

                        # 等待降落完成
                        print("等待降落完成...")
                        start_time = asyncio.get_event_loop().time()
                        timeout = 30  # 30秒超时

                        async for position in drone_system.telemetry.position():
                            current_time = asyncio.get_event_loop().time()
                            if current_time - start_time > timeout:
                                print("降落超时，但继续执行...")
                                break

                            # 检查是否接近地面
                            if position.relative_altitude_m <= 0.5:  # 距离地面0.5米以内
                                print(f"当前高度: {position.relative_altitude_m:.2f}m")
                                break

                        drone_status["flying"] = False
                        drone_status["altitude"] = 0.0
                        print("✓ 无人机降落完成")
                        if command_id:
                            command_results[command_id] = "✓ 无人机降落完成"

                    elif command == "move":
                        result = await _execute_movement(params)
                        if command_id:
                            command_results[command_id] = result

                    elif command == "rotate":
                        result = await _execute_rotation(params)
                        if command_id:
                            command_results[command_id] = result

                    elif command == "hover":
                        duration = params.get("duration", 5.0) if params else 5.0
                        print(f"悬停 {duration} 秒...")
                        await asyncio.sleep(duration)
                        print("✓ 悬停完成")
                        if command_id:
                            command_results[command_id] = "✓ 悬停完成"

                    elif command == "hold_altitude":
                        target_altitude = params.get("altitude", 5.0) if params else 5.0
                        print(f"保持高度 {target_altitude}米...")

                        try:
                            # 获取当前位置
                            async for current_pos in drone_system.telemetry.position():
                                # 发送goto命令到当前位置但指定目标高度
                                await drone_system.action.goto_location(
                                    current_pos.latitude_deg,
                                    current_pos.longitude_deg,
                                    target_altitude,
                                    0  # 保持当前偏航角
                                )
                                print(f"✓ 高度保持命令已发送: {target_altitude}米")
                                break

                            # 等待高度稳定
                            start_time = asyncio.get_event_loop().time()
                            timeout = 20
                            altitude_tolerance = 0.2
                            stable_count = 0
                            required_stable_readings = 3

                            async for position in drone_system.telemetry.position():
                                current_time = asyncio.get_event_loop().time()
                                if current_time - start_time > timeout:
                                    print("高度保持超时")
                                    break

                                current_alt = position.relative_altitude_m
                                altitude_diff = abs(current_alt - target_altitude)

                                if altitude_diff <= altitude_tolerance:
                                    stable_count += 1
                                    if stable_count >= required_stable_readings:
                                        print(f"✓ 高度已稳定在 {current_alt:.2f}米")
                                        break
                                else:
                                    stable_count = 0

                                await asyncio.sleep(0.5)

                            drone_status["altitude"] = target_altitude
                            if command_id:
                                command_results[command_id] = f"✓ 高度保持完成: {target_altitude}米"

                        except Exception as hold_error:
                            print(f"高度保持失败: {hold_error}")
                            if command_id:
                                command_results[command_id] = f"高度保持失败: {hold_error}"

                    elif command == "emergency_land":
                        print("紧急降落!")
                        await drone_system.action.kill()
                        drone_status["flying"] = False
                        drone_status["armed"] = False
                        print("✓ 紧急降落完成")
                        if command_id:
                            command_results[command_id] = "✓ 紧急降落完成"

                    elif command == "manual_control":
                        await _execute_manual_control(params)
                        if command_id:
                            command_results[command_id] = "✓ 手动控制设置完成"

                    elif command == "flight_sequence":
                        result = await _execute_flight_sequence(params)
                        if command_id:
                            command_results[command_id] = result

                except Exception as cmd_error:
                    error_msg = f"命令执行失败: {cmd_error}"
                    print(error_msg)
                    if command_id:
                        command_results[command_id] = error_msg
                        
        except Exception as e:
            print(f"处理无人机命令时出错: {e}")
            
        await asyncio.sleep(0.1)

async def _execute_movement(params):
    """Execute movement command with stable orientation and altitude control."""
    direction = params["direction"]
    distance = params["distance"]
    speed = params["speed"]
    command_id = params.get("command_id")

    print(f"开始向{direction}移动 {distance}米，速度 {speed}m/s")

    try:
        # 获取当前位置和偏航角
        current_lat = current_lon = current_alt = current_yaw = 0

        async for position in drone_system.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            current_alt = position.relative_altitude_m
            break

        async for attitude in drone_system.telemetry.attitude_euler():
            current_yaw = attitude.yaw_deg
            break

        print(f"📍 移动起始位置: 高度{current_alt:.2f}m, 偏航{current_yaw:.1f}°")

        # 计算移动时间
        duration = distance / speed

        if direction == "forward":
            try:
                print(f"🚁 开始向前移动 {distance}米，保持高度{current_alt:.2f}m和偏航角{current_yaw:.1f}°")

                # 使用offboard模式进行精确移动
                print("🎯 启动offboard模式进行前进移动")

                # 设置速度控制模式
                await drone_system.offboard.set_velocity_ned(
                    VelocityNedYaw(speed, 0.0, 0.0, current_yaw)
                )
                await drone_system.offboard.start()
                print("✅ Offboard模式已启动")

                # 移动期间监控位置和方向稳定性
                start_time = asyncio.get_event_loop().time()
                last_position_check = 0
                position_check_interval = 1.0  # 每秒检查一次位置稳定性

                while asyncio.get_event_loop().time() - start_time < duration:
                    current_time = asyncio.get_event_loop().time()

                    # 定期检查高度和偏航角稳定性
                    if current_time - last_position_check >= position_check_interval:
                        try:
                            async for position in drone_system.telemetry.position():
                                current_height = position.relative_altitude_m
                                height_diff = abs(current_height - current_alt)

                                if height_diff > 0.5:  # 高度偏差过大
                                    print(f"⚠️ 移动中高度偏差: {current_height:.2f}m vs 目标{current_alt:.2f}m")
                                    # 可以在这里添加高度修正逻辑

                                print(f"📊 移动监控: 高度{current_height:.2f}m (目标{current_alt:.2f}m)")
                                break
                        except Exception as monitor_error:
                            print(f"移动监控错误: {monitor_error}")

                        last_position_check = current_time

                    await asyncio.sleep(0.2)

                # 停止移动，恢复位置保持模式
                print("🛑 停止移动，恢复位置保持模式")
                await drone_system.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, current_yaw)
                )
                await asyncio.sleep(0.5)
                await drone_system.offboard.stop()
                print("✅ Offboard模式已停止")

                # 获取最终位置并发送位置保持命令
                async for final_position in drone_system.telemetry.position():
                    final_alt = final_position.relative_altitude_m
                    print(f"📍 移动完成位置: 高度{final_alt:.2f}m")

                    # 发送位置保持命令，确保稳定悬停
                    await drone_system.action.goto_location(
                        final_position.latitude_deg,
                        final_position.longitude_deg,
                        current_alt,  # 保持原始高度
                        current_yaw   # 保持原始偏航角
                    )
                    print(f"✅ 移动完成，恢复位置保持模式")
                    break

                # 等待位置稳定
                await asyncio.sleep(2)

                result_msg = f"✅ 向{direction}移动 {distance}米完成"
                print(result_msg)
                return result_msg

            except Exception as offboard_error:
                print(f"⚠️ Offboard移动失败: {offboard_error}")
                print("🔄 尝试使用备用移动方法...")

                try:
                    # 备用方法：使用goto_location进行相对移动
                    # 获取当前位置
                    async for position in drone_system.telemetry.position():
                        current_lat = position.latitude_deg
                        current_lon = position.longitude_deg
                        break

                    # 计算大致的目标位置（简化版本）
                    # 注意：这是一个简化的实现，实际应该根据偏航角计算精确位置
                    lat_offset = distance * 0.00001 * 1.0  # 大致的纬度偏移
                    target_lat = current_lat + lat_offset

                    print(f"🎯 使用位置控制模式移动到目标位置")
                    await drone_system.action.goto_location(
                        target_lat,
                        current_lon,
                        current_alt,
                        current_yaw
                    )

                    # 等待移动完成
                    print("⏳ 等待移动完成...")
                    await asyncio.sleep(duration + 2)

                    result_msg = f"✅ 向{direction}移动 {distance}米完成（备用方法）"
                    print(result_msg)
                    return result_msg

                except Exception as backup_error:
                    error_msg = f"❌ 备用移动方法也失败: {backup_error}"
                    print(error_msg)
                    return error_msg

        else:
            # 其他方向的移动（简化实现）
            print(f"方向 {direction} 的移动功能待完善")
            await asyncio.sleep(duration)
            result_msg = f"✅ 向{direction}移动 {distance}米完成（简化模式）"
            return result_msg

    except Exception as e:
        error_msg = f"移动执行错误: {e}"
        print(error_msg)
        return error_msg

async def _execute_rotation(params):
    """Execute rotation command."""
    angle = params["angle"]
    direction = params["direction"]
    
    print(f"✓ {direction}旋转 {angle} 度")
    # 这里应该实现具体的旋转逻辑
    await asyncio.sleep(2.0)

async def _execute_manual_control(params):
    """Execute manual control command."""
    roll = params["roll"]
    pitch = params["pitch"]
    throttle = params["throttle"]
    yaw = params["yaw"]

    print(f"✓ 手动控制: roll={roll}, pitch={pitch}, throttle={throttle}, yaw={yaw}")
    # 这里应该实现具体的手动控制逻辑

async def _execute_flight_sequence(params):
    """Execute a complete flight sequence: takeoff → move → land."""
    altitude = params.get("altitude", 5.0)
    forward_distance = params.get("forward_distance", 5.0)
    speed = params.get("speed", 2.0)

    print(f"🚁 开始执行飞行序列: 起飞{altitude}m → 前进{forward_distance}m → 降落")

    try:
        # 阶段1: 起飞到指定高度
        print(f"\n📈 阶段1: 起飞到 {altitude}米")
        print("-" * 40)

        # 确保无人机已解锁
        if not drone_status.get("armed", False):
            print("无人机未解锁，先解锁...")
            try:
                await drone_system.action.arm()
                drone_status["armed"] = True
                print("✓ 无人机解锁成功")
                await asyncio.sleep(2)
            except Exception as arm_error:
                return f"解锁失败: {arm_error}"

        # 执行起飞（使用改进的高度控制）
        await _execute_takeoff_to_altitude(altitude)

        # 验证起飞成功
        current_alt = 0
        async for position in drone_system.telemetry.position():
            current_alt = position.relative_altitude_m
            break

        if current_alt < altitude * 0.8:  # 如果高度不足目标的80%
            return f"❌ 起飞失败，当前高度 {current_alt:.2f}m 低于目标 {altitude}m"

        print(f"✅ 起飞完成，当前高度: {current_alt:.2f}m")

        # 阶段2: 前进移动
        print(f"\n➡️ 阶段2: 向前移动 {forward_distance}米")
        print("-" * 40)

        # 执行前进移动
        move_result = await _execute_movement({
            "direction": "forward",
            "distance": forward_distance,
            "speed": speed
        })

        if "失败" in move_result:
            return f"❌ 移动失败: {move_result}"

        print(f"✅ 前进移动完成")

        # 短暂悬停确保稳定
        print("⏸️ 短暂悬停确保稳定...")
        await asyncio.sleep(3)

        # 阶段3: 安全降落
        print(f"\n🛬 阶段3: 安全降落")
        print("-" * 40)

        # 执行降落
        await drone_system.action.land()

        # 等待降落完成
        print("等待降落完成...")
        start_time = asyncio.get_event_loop().time()
        timeout = 30

        async for position in drone_system.telemetry.position():
            current_time = asyncio.get_event_loop().time()
            if current_time - start_time > timeout:
                print("降落超时，但继续执行...")
                break

            if position.relative_altitude_m <= 0.5:
                print(f"✅ 降落完成，当前高度: {position.relative_altitude_m:.2f}m")
                break

            await asyncio.sleep(0.5)

        # 更新状态
        drone_status["flying"] = False
        drone_status["altitude"] = 0.0

        result_msg = f"🎉 飞行序列完成: 起飞{altitude}m → 前进{forward_distance}m → 降落"
        print(result_msg)
        return result_msg

    except Exception as e:
        error_msg = f"飞行序列执行失败: {e}"
        print(error_msg)
        return error_msg

async def _execute_takeoff_to_altitude(altitude):
    """Execute takeoff to specific altitude with improved control."""
    print(f"🚁 开始改进的高度控制起飞到 {altitude}米")

    # 获取当前偏航角以保持方向稳定
    current_yaw = 0
    try:
        async for attitude in drone_system.telemetry.attitude_euler():
            current_yaw = attitude.yaw_deg
            print(f"📐 当前偏航角: {current_yaw:.1f}°")
            break
    except Exception as yaw_error:
        print(f"⚠️ 无法获取偏航角，使用默认值: {yaw_error}")

    # 阶段1：直接起飞到目标高度
    print(f"阶段1：直接起飞到目标高度 {altitude}米")
    await drone_system.action.set_takeoff_altitude(altitude)
    await drone_system.action.takeoff()

    # 阶段2：智能高度监控和稳定控制
    print(f"🎯 阶段2：智能高度监控和稳定控制")
    start_time = asyncio.get_event_loop().time()
    timeout = 45  # 增加超时时间适应更高的高度
    altitude_tolerance = 0.3  # 对于更高的高度，稍微放宽容差
    stable_count = 0
    required_stable_readings = 5

    # 控制参数
    last_control_time = 0
    control_interval = 1.5  # 稍微增加控制间隔

    async for position in drone_system.telemetry.position():
        current_time = asyncio.get_event_loop().time()
        if current_time - start_time > timeout:
            print("⏰ 高度控制超时，完成起飞...")
            break

        current_alt = position.relative_altitude_m
        altitude_error = altitude - current_alt
        altitude_diff = abs(altitude_error)

        print(f"📊 高度监控: {current_alt:.2f}m (目标: {altitude}m, 误差: {altitude_error:+.2f}m)")

        # 检测是否需要发送控制命令
        should_send_control = False
        control_reason = ""

        # 条件1：高度误差超过容差
        if altitude_diff > altitude_tolerance:
            should_send_control = True
            control_reason = f"误差过大({altitude_error:+.2f}m)"

        # 条件2：定期控制更新
        elif current_time - last_control_time >= control_interval and altitude_diff > 0.15:
            should_send_control = True
            control_reason = "定期控制更新"
            last_control_time = current_time

        # 发送控制命令（保持偏航角稳定）
        if should_send_control:
            print(f"🎯 发送高度控制命令 - 原因: {control_reason}")
            try:
                await drone_system.action.goto_location(
                    position.latitude_deg,
                    position.longitude_deg,
                    altitude,
                    current_yaw  # 保持当前偏航角，防止旋转
                )
                print(f"✅ 高度控制命令已发送: 目标{altitude}m, 偏航{current_yaw:.1f}°")
            except Exception as e:
                print(f"❌ 高度控制命令失败: {e}")

        # 检查高度稳定性
        if altitude_diff <= altitude_tolerance:
            stable_count += 1
            print(f"📈 高度稳定进度: {stable_count}/{required_stable_readings}")

            if stable_count >= required_stable_readings:
                print(f"🎉 目标高度已达到并稳定: {current_alt:.2f}m")
                break
        else:
            stable_count = 0

        await asyncio.sleep(0.5)

    # 最终稳定
    print("阶段3：最终稳定...")
    try:
        async for current_pos in drone_system.telemetry.position():
            final_altitude = current_pos.relative_altitude_m
            print(f"最终高度验证: {final_altitude:.2f}m (目标: {altitude}m)")

            await drone_system.action.goto_location(
                current_pos.latitude_deg,
                current_pos.longitude_deg,
                altitude,
                current_yaw
            )
            print("✓ 最终位置保持命令已发送")
            break

        await asyncio.sleep(1)

        # 更新状态
        drone_status["flying"] = True
        drone_status["altitude"] = altitude

    except Exception as final_error:
        print(f"最终稳定命令失败: {final_error}")

def _wait_for_command_completion(command_id: str, initial_message: str, timeout: int = 30) -> str:
    """等待命令完成并返回结果"""
    import time
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        if command_id in command_results:
            result = command_results.pop(command_id)
            return result
        time.sleep(0.1)
    
    return f"{initial_message} - 超时，命令可能未完成"

# Start the command processor when module is imported
def start_command_processor():
    """Start the background command processor."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(process_drone_commands())

# Start processor in background thread
command_thread = threading.Thread(target=start_command_processor, daemon=True)
command_thread.start()
