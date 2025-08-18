#!/usr/bin/env python3
"""
测试自定义高度起飞功能
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_simple

def test_altitude_takeoff():
    """测试自定义高度起飞功能"""
    print("=== 自定义高度起飞功能测试 ===")
    
    # 1. 检查连接状态
    print("\n1. 检查连接状态...")
    connect_result = uav_simple.connect_drone.invoke("")
    print(f"连接结果: {connect_result}")
    
    # 2. 检查当前状态
    print("\n2. 检查当前状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 3. 解锁
    print("\n3. 解锁无人机...")
    arm_result = uav_simple.arm_drone.invoke("")
    print(f"解锁结果: {arm_result}")
    time.sleep(3)
    
    # 4. 测试起飞到1米高度
    print("\n4. 测试起飞到1米高度...")
    takeoff_result = uav_simple.takeoff_drone_to_altitude.invoke({"altitude": 1.0})
    print(f"起飞结果: {takeoff_result}")
    time.sleep(8)
    
    # 5. 检查起飞后状态
    print("\n5. 检查起飞后状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 6. 等待一下观察高度
    print("\n6. 等待观察高度...")
    time.sleep(5)
    
    # 7. 降落
    print("\n7. 降落...")
    land_result = uav_simple.land_drone.invoke("")
    print(f"降落结果: {land_result}")
    time.sleep(10)
    
    # 8. 检查最终状态
    print("\n8. 检查最终状态...")
    final_status = uav_simple.get_drone_status.invoke("")
    print(f"最终状态: {final_status}")
    
    print("\n=== 自定义高度起飞功能测试完成 ===")
    
    # 总结
    if "successful" in takeoff_result.lower():
        print("✅ 自定义高度起飞功能测试成功！")
        print("请检查PX4终端，应该显示起飞高度为1.0米而不是20米。")
        return True
    else:
        print("❌ 自定义高度起飞功能有问题。")
        return False

def main():
    """主函数"""
    try:
        success = test_altitude_takeoff()
        if success:
            print("\n🎉 自定义高度起飞功能工作正常！")
            print("现在可以指定具体的起飞高度了。")
        else:
            print("\n⚠️ 自定义高度起飞功能有问题，需要进一步调试。")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
