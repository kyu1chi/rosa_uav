#!/usr/bin/env python3
"""
测试MAVSDK offboard移动功能
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_simple, uav_offboard

def test_offboard_movement():
    """测试MAVSDK offboard移动功能"""
    print("=== MAVSDK Offboard移动功能测试 ===")
    
    # 1. 检查连接状态
    print("\n1. 检查连接状态...")
    connect_result = uav_simple.connect_drone.invoke("")
    print(f"连接结果: {connect_result}")
    
    # 2. 检查当前状态
    print("\n2. 检查当前状态...")
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
    
    # 6. 测试悬停
    print("\n6. 测试悬停...")
    print("请观察Gazebo仿真环境中的无人机是否稳定悬停...")
    hover_result = uav_offboard.hover_drone.invoke("")
    print(f"悬停结果: {hover_result}")
    time.sleep(2)
    
    # 7. 测试offboard向前移动（5米，便于观察）
    print("\n7. 测试offboard向前移动（5米）...")
    print("请观察Gazebo仿真环境中的无人机是否向前移动...")
    forward_move_result = uav_offboard.move_drone_offboard.invoke({"direction": "forward", "distance": 5.0})
    print(f"向前移动结果: {forward_move_result}")
    time.sleep(2)
    
    # 8. 测试向右移动
    print("\n8. 测试向右移动（3米）...")
    print("请观察Gazebo仿真环境中的无人机是否向右移动...")
    right_move_result = uav_offboard.move_drone_offboard.invoke({"direction": "right", "distance": 3.0})
    print(f"向右移动结果: {right_move_result}")
    time.sleep(2)
    
    # 9. 测试向后移动回来
    print("\n9. 测试向后移动回来（5米）...")
    print("请观察Gazebo仿真环境中的无人机是否向后移动...")
    backward_move_result = uav_offboard.move_drone_offboard.invoke({"direction": "backward", "distance": 5.0})
    print(f"向后移动结果: {backward_move_result}")
    time.sleep(2)
    
    # 10. 测试向左移动回来
    print("\n10. 测试向左移动回来（3米）...")
    print("请观察Gazebo仿真环境中的无人机是否向左移动...")
    left_move_result = uav_offboard.move_drone_offboard.invoke({"direction": "left", "distance": 3.0})
    print(f"向左移动结果: {left_move_result}")
    time.sleep(2)
    
    # 11. 检查移动后状态
    print("\n11. 检查移动后状态...")
    final_status = uav_simple.get_drone_status.invoke("")
    print(f"移动后状态: {final_status}")
    
    # 12. 降落
    print("\n12. 降落...")
    land_result = uav_simple.land_drone.invoke("")
    print(f"降落结果: {land_result}")
    time.sleep(10)
    
    print("\n=== MAVSDK Offboard移动功能测试完成 ===")
    print("\n请确认在Gazebo仿真环境中观察到了以下移动:")
    print("1. 无人机稳定悬停")
    print("2. 无人机向前移动了5米")
    print("3. 无人机向右移动了3米") 
    print("4. 无人机向后移动了5米")
    print("5. 无人机向左移动了3米")
    print("6. 无人机成功降落")
    
    # 总结
    if "successfully" in forward_move_result.lower():
        print("\n✅ MAVSDK Offboard移动命令执行成功！")
        print("如果在Gazebo中看到了实际移动，说明offboard模式工作正常。")
        return True
    else:
        print("\n❌ MAVSDK Offboard移动命令执行失败。")
        return False

def main():
    """主函数"""
    try:
        success = test_offboard_movement()
        if success:
            print("\n🎉 MAVSDK Offboard移动功能测试完成！")
            print("现在应该能在Gazebo仿真环境中看到实际的无人机移动。")
        else:
            print("\n⚠️ MAVSDK Offboard移动功能有问题，需要进一步调试。")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
