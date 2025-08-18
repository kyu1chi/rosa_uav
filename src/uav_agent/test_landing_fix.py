#!/usr/bin/env python3
"""
测试修复后的降落功能
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_simple

def test_landing_functionality():
    """测试降落功能"""
    print("=== 降落功能测试 ===")
    
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
        time.sleep(10)  # 等待起飞完成
    
    # 5. 检查起飞后状态
    print("\n5. 检查起飞后状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 6. 测试普通降落
    print("\n6. 测试普通降落...")
    land_result = uav_simple.land_drone.invoke("")
    print(f"降落结果: {land_result}")
    
    # 7. 等待降落完成
    print("\n7. 等待降落完成...")
    time.sleep(15)
    
    # 8. 检查降落后状态
    print("\n8. 检查降落后状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 9. 如果普通降落失败，测试返航降落
    if "In Air=True" in status:
        print("\n9. 普通降落可能失败，尝试返航降落...")
        rtl_result = uav_simple.return_to_launch.invoke("")
        print(f"返航降落结果: {rtl_result}")
        
        time.sleep(20)
        
        print("\n10. 检查返航降落后状态...")
        status = uav_simple.get_drone_status.invoke("")
        print(f"状态: {status}")
    
    print("\n=== 降落功能测试完成 ===")
    
    # 总结
    final_status = uav_simple.get_drone_status.invoke("")
    if "In Air=False" in final_status:
        print("✅ 降落测试成功！无人机已着陆。")
    else:
        print("❌ 降落测试可能失败，无人机仍在空中。")
    
    return True

def main():
    """主函数"""
    try:
        test_landing_functionality()
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
