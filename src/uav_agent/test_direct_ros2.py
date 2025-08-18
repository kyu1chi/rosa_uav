#!/usr/bin/env python3
"""
直接测试ROS2命令发送
"""

import subprocess
import time

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

def test_direct_ros2_movement():
    """直接测试ROS2移动命令"""
    print("=== 直接ROS2移动命令测试 ===")
    
    # 1. 检查当前飞行模式
    print("\n1. 检查当前飞行模式...")
    mode_cmd = "ros2 topic echo --once /fmu/out/vehicle_control_mode"
    success, stdout, stderr = run_ros2_command(mode_cmd, timeout=5)
    print(f"飞行模式: {stdout[:200] if stdout else stderr}")
    
    # 2. 发送offboard控制模式
    print("\n2. 发送offboard控制模式...")
    offboard_cmd = """ros2 topic pub --once /fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode "{
        position: false,
        velocity: true,
        acceleration: false,
        attitude: false,
        body_rate: false,
        thrust_and_torque: false,
        direct_actuator: false,
        timestamp: 0
    }" """
    
    success1, stdout1, stderr1 = run_ros2_command(offboard_cmd)
    print(f"Offboard控制模式结果: {'成功' if success1 else '失败'}")
    if not success1:
        print(f"错误: {stderr1}")
    
    # 3. 发送向前速度命令
    print("\n3. 发送向前速度命令...")
    velocity_cmd = """ros2 topic pub --once /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{
        position: [NaN, NaN, NaN],
        velocity: [2.0, 0.0, 0.0],
        acceleration: [NaN, NaN, NaN],
        jerk: [NaN, NaN, NaN],
        yaw: NaN,
        yawspeed: NaN,
        timestamp: 0
    }" """
    
    success2, stdout2, stderr2 = run_ros2_command(velocity_cmd)
    print(f"速度命令结果: {'成功' if success2 else '失败'}")
    if not success2:
        print(f"错误: {stderr2}")
    
    # 4. 等待观察
    print("\n4. 等待5秒观察无人机移动...")
    time.sleep(5)
    
    # 5. 发送停止命令
    print("\n5. 发送停止命令...")
    stop_cmd = """ros2 topic pub --once /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{
        position: [NaN, NaN, NaN],
        velocity: [0.0, 0.0, 0.0],
        acceleration: [NaN, NaN, NaN],
        jerk: [NaN, NaN, NaN],
        yaw: NaN,
        yawspeed: NaN,
        timestamp: 0
    }" """
    
    success3, stdout3, stderr3 = run_ros2_command(stop_cmd)
    print(f"停止命令结果: {'成功' if success3 else '失败'}")
    if not success3:
        print(f"错误: {stderr3}")
    
    # 6. 检查位置变化
    print("\n6. 检查位置...")
    position_cmd = "ros2 topic echo --once /fmu/out/vehicle_local_position"
    success4, stdout4, stderr4 = run_ros2_command(position_cmd, timeout=5)
    print(f"位置信息: {stdout4[:300] if stdout4 else stderr4}")
    
    print("\n=== 直接ROS2移动命令测试完成 ===")
    
    if success1 and success2 and success3:
        print("✅ 所有ROS2命令发送成功！")
        print("请检查Gazebo仿真环境中无人机是否有移动。")
        return True
    else:
        print("❌ 部分ROS2命令发送失败。")
        return False

def main():
    """主函数"""
    try:
        success = test_direct_ros2_movement()
        if success:
            print("\n🎉 直接ROS2命令测试完成！")
            print("如果无人机仍然没有移动，可能需要检查PX4的offboard模式配置。")
        else:
            print("\n⚠️ ROS2命令发送有问题。")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
