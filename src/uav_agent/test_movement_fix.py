#!/usr/bin/env python3
"""
测试修复后的移动功能
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_simple

def test_movement_functionality():
    """测试移动功能"""
    print("=== 移动功能测试 ===")
    
    # 1. 检查连接状态
    print("\n1. 检查连接状态...")
    connect_result = uav_simple.connect_drone.invoke("")
    print(f"连接结果: {connect_result}")
    
    # 2. 检查当前状态（实时同步）
    print("\n2. 检查当前状态（实时同步）...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 3. 如果没有解锁，先解锁
    if "Armed=False" in status:
        print("\n3. 解锁无人机...")
        arm_result = uav_simple.arm_drone.invoke("")
        print(f"解锁结果: {arm_result}")
        time.sleep(3)
    
    # 4. 如果没有起飞，先起飞
    if "In Air=False" in status:
        print("\n4. 起飞...")
        takeoff_result = uav_simple.takeoff_drone.invoke("")
        print(f"起飞结果: {takeoff_result}")
        time.sleep(8)
    
    # 5. 再次检查状态
    print("\n5. 再次检查状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 6. 测试短距离移动（2秒）
    print("\n6. 测试短距离移动（左2秒）...")
    move_result = uav_simple.move_drone.invoke({"direction": "left", "duration": 2.0})
    print(f"短距离移动结果: {move_result}")
    time.sleep(1)
    
    # 7. 测试距离移动（左10米）
    print("\n7. 测试距离移动（左10米）...")
    distance_move_result = uav_simple.move_drone_distance.invoke({"direction": "left", "distance": 10.0})
    print(f"距离移动结果: {distance_move_result}")
    time.sleep(2)
    
    # 8. 测试向右移动回来
    print("\n8. 测试向右移动回来（右10米）...")
    return_move_result = uav_simple.move_drone_distance.invoke({"direction": "right", "distance": 10.0})
    print(f"返回移动结果: {return_move_result}")
    time.sleep(2)
    
    # 9. 检查移动后状态
    print("\n9. 检查移动后状态...")
    final_status = uav_simple.get_drone_status.invoke("")
    print(f"移动后状态: {final_status}")
    
    # 10. 降落
    print("\n10. 降落...")
    land_result = uav_simple.land_drone.invoke("")
    print(f"降落结果: {land_result}")
    time.sleep(10)
    
    # 11. 最终状态检查
    print("\n11. 最终状态检查...")
    final_status = uav_simple.get_drone_status.invoke("")
    print(f"最终状态: {final_status}")
    
    print("\n=== 移动功能测试完成 ===")
    
    # 总结
    if "successfully" in distance_move_result.lower():
        print("✅ 距离移动功能测试成功！")
        return True
    else:
        print("❌ 距离移动功能仍有问题。")
        return False

def main():
    """主函数"""
    try:
        success = test_movement_functionality()
        if success:
            print("\n🎉 移动功能修复成功！现在可以正确处理距离移动命令。")
        else:
            print("\n⚠️ 移动功能仍有问题，需要进一步调试。")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
