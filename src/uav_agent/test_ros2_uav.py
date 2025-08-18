#!/usr/bin/env python3
"""
测试基于ROS2的UAV控制
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_ros2

def test_ros2_uav_system():
    """测试基于ROS2的UAV系统"""
    print("=== 基于ROS2的UAV系统测试 ===")
    
    # 1. 检查连接状态
    print("\n1. 检查连接状态...")
    connect_result = uav_ros2.connect_drone.invoke("")
    print(f"连接结果: {connect_result}")
    
    # 2. 检查状态
    print("\n2. 检查无人机状态...")
    status = uav_ros2.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 3. 设置飞行模式为手动
    print("\n3. 设置飞行模式...")
    mode_result = uav_ros2.set_flight_mode.invoke("manual")
    print(f"模式设置结果: {mode_result}")
    
    time.sleep(2)
    
    # 4. 解锁
    print("\n4. 解锁无人机...")
    arm_result = uav_ros2.arm_drone.invoke("")
    print(f"解锁结果: {arm_result}")
    
    time.sleep(3)
    
    # 5. 检查解锁后状态
    print("\n5. 检查解锁后状态...")
    status = uav_ros2.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 6. 起飞
    print("\n6. 起飞...")
    takeoff_result = uav_ros2.takeoff_drone.invoke("")
    print(f"起飞结果: {takeoff_result}")
    
    time.sleep(8)
    
    # 7. 检查起飞后状态
    print("\n7. 检查起飞后状态...")
    status = uav_ros2.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 8. 降落
    print("\n8. 降落...")
    land_result = uav_ros2.land_drone.invoke("")
    print(f"降落结果: {land_result}")
    
    time.sleep(8)
    
    # 9. 锁定
    print("\n9. 锁定...")
    disarm_result = uav_ros2.disarm_drone.invoke("")
    print(f"锁定结果: {disarm_result}")
    
    # 10. 最终状态检查
    print("\n10. 最终状态检查...")
    status = uav_ros2.get_drone_status.invoke("")
    print(f"最终状态: {status}")
    
    print("\n=== 基于ROS2的UAV系统测试完成 ===")
    return True

def main():
    """主函数"""
    try:
        test_ros2_uav_system()
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
