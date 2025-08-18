#!/usr/bin/env python3
"""
测试完整的飞行序列：起飞 -> 移动 -> 降落
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_simple

def test_complete_flight():
    """测试完整的飞行序列"""
    print("=== 完整飞行序列测试 ===")
    
    # 1. 检查连接
    print("\n1. 检查连接...")
    connect_result = uav_simple.connect_drone.invoke("")
    print(f"连接: {connect_result}")
    
    # 2. 解锁
    print("\n2. 解锁无人机...")
    arm_result = uav_simple.arm_drone.invoke("")
    print(f"解锁: {arm_result}")
    time.sleep(3)
    
    # 3. 起飞
    print("\n3. 起飞...")
    takeoff_result = uav_simple.takeoff_drone.invoke("")
    print(f"起飞: {takeoff_result}")
    time.sleep(8)
    
    # 4. 检查状态
    print("\n4. 检查起飞后状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 5. 向前移动
    print("\n5. 向前移动3秒...")
    move_result = uav_simple.move_drone.invoke({"direction": "forward", "duration": 3.0})
    print(f"向前移动: {move_result}")
    time.sleep(1)
    
    # 6. 向右移动
    print("\n6. 向右移动2秒...")
    move_result = uav_simple.move_drone.invoke({"direction": "right", "duration": 2.0})
    print(f"向右移动: {move_result}")
    time.sleep(1)
    
    # 7. 向上移动
    print("\n7. 向上移动2秒...")
    move_result = uav_simple.move_drone.invoke({"direction": "up", "duration": 2.0})
    print(f"向上移动: {move_result}")
    time.sleep(1)
    
    # 8. 向后移动
    print("\n8. 向后移动3秒...")
    move_result = uav_simple.move_drone.invoke({"direction": "backward", "duration": 3.0})
    print(f"向后移动: {move_result}")
    time.sleep(1)
    
    # 9. 降落
    print("\n9. 降落...")
    land_result = uav_simple.land_drone.invoke("")
    print(f"降落: {land_result}")
    time.sleep(15)
    
    # 10. 检查最终状态
    print("\n10. 检查最终状态...")
    final_status = uav_simple.get_drone_status.invoke("")
    print(f"最终状态: {final_status}")
    
    # 11. 锁定
    print("\n11. 锁定...")
    disarm_result = uav_simple.disarm_drone.invoke("")
    print(f"锁定: {disarm_result}")
    
    print("\n=== 完整飞行序列测试完成 ===")
    
    # 总结
    if "In Air=False" in final_status:
        print("✅ 完整飞行测试成功！无人机完成了起飞、移动、降落的完整序列。")
    else:
        print("⚠️ 飞行测试部分成功，但无人机可能仍在空中。")
    
    return True

def main():
    """主函数"""
    try:
        test_complete_flight()
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
