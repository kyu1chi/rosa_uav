#!/usr/bin/env python3
"""
测试简化版UAV系统
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_simple

def test_simple_uav_system():
    """测试简化版UAV系统"""
    print("=== 简化版UAV系统测试 ===")
    
    # 1. 检查连接状态
    print("\n1. 检查连接状态...")
    connect_result = uav_simple.connect_drone.invoke("")
    print(f"连接结果: {connect_result}")
    
    # 2. 检查状态
    print("\n2. 检查无人机状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 3. 解锁
    print("\n3. 解锁无人机...")
    arm_result = uav_simple.arm_drone.invoke("")
    print(f"解锁结果: {arm_result}")
    
    time.sleep(3)
    
    # 4. 检查解锁后状态
    print("\n4. 检查解锁后状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 5. 起飞
    print("\n5. 起飞...")
    takeoff_result = uav_simple.takeoff_drone.invoke("")
    print(f"起飞结果: {takeoff_result}")
    
    time.sleep(8)
    
    # 6. 检查起飞后状态
    print("\n6. 检查起飞后状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 7. 降落
    print("\n7. 降落...")
    land_result = uav_simple.land_drone.invoke("")
    print(f"降落结果: {land_result}")
    
    time.sleep(8)
    
    # 8. 锁定
    print("\n8. 锁定...")
    disarm_result = uav_simple.disarm_drone.invoke("")
    print(f"锁定结果: {disarm_result}")
    
    # 9. 最终状态检查
    print("\n9. 最终状态检查...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"最终状态: {status}")
    
    print("\n=== 简化版UAV系统测试完成 ===")
    return True

def main():
    """主函数"""
    try:
        test_simple_uav_system()
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
