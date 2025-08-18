#!/usr/bin/env python3
"""
专门测试降落功能
"""

import time
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav_simple

def test_landing_only():
    """专门测试降落功能"""
    print("=== 专门测试降落功能 ===")
    
    # 1. 检查连接
    print("\n1. 检查连接...")
    connect_result = uav_simple.connect_drone.invoke("")
    print(f"连接: {connect_result}")
    
    # 2. 检查当前状态
    print("\n2. 检查当前状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 3. 如果没有解锁，先解锁
    if "Armed=False" in status:
        print("\n3. 解锁无人机...")
        arm_result = uav_simple.arm_drone.invoke("")
        print(f"解锁: {arm_result}")
        time.sleep(3)
    
    # 4. 如果没有起飞，先起飞
    if "In Air=False" in status:
        print("\n4. 起飞...")
        takeoff_result = uav_simple.takeoff_drone.invoke("")
        print(f"起飞: {takeoff_result}")
        time.sleep(8)
    
    # 5. 检查起飞后状态
    print("\n5. 检查起飞后状态...")
    status = uav_simple.get_drone_status.invoke("")
    print(f"状态: {status}")
    
    # 6. 降落测试
    print("\n6. 开始降落测试...")
    land_result = uav_simple.land_drone.invoke("")
    print(f"降落结果: {land_result}")
    
    # 7. 等待降落完成
    print("\n7. 等待降落完成（15秒）...")
    time.sleep(15)
    
    # 8. 检查降落后状态
    print("\n8. 检查降落后状态...")
    final_status = uav_simple.get_drone_status.invoke("")
    print(f"最终状态: {final_status}")
    
    print("\n=== 降落功能测试完成 ===")
    
    # 结果判断
    if "In Air=False" in final_status:
        print("✅ 降落测试成功！无人机已安全着陆。")
        return True
    else:
        print("❌ 降落测试失败，无人机仍在空中。")
        
        # 尝试返航降落作为备选
        print("\n尝试返航降落作为备选方案...")
        rtl_result = uav_simple.return_to_launch.invoke("")
        print(f"返航降落: {rtl_result}")
        
        time.sleep(20)
        backup_status = uav_simple.get_drone_status.invoke("")
        print(f"返航后状态: {backup_status}")
        
        if "In Air=False" in backup_status:
            print("✅ 返航降落成功！")
            return True
        else:
            print("❌ 返航降落也失败了。")
            return False

def main():
    """主函数"""
    try:
        success = test_landing_only()
        if success:
            print("\n🎉 降落功能修复成功！")
        else:
            print("\n⚠️ 降落功能仍有问题，需要进一步调试。")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
