#!/usr/bin/env python3
"""
测试UAV连接和基本功能的脚本
"""

import asyncio
import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from uav_agent.tools import uav as uav_tools

async def test_uav_system():
    """测试UAV系统的基本功能"""
    print("=== UAV系统测试开始 ===")
    
    # 1. 初始化UAV系统
    print("\n1. 初始化UAV系统...")
    success = await uav_tools.initialize_uav_system()
    if not success:
        print("❌ UAV系统初始化失败")
        return False
    
    print("✅ UAV系统初始化成功")
    
    # 等待一下让系统稳定
    await asyncio.sleep(2)
    
    # 2. 检查连接状态
    print("\n2. 检查连接状态...")
    status = uav_tools.get_drone_status.invoke("")
    print(f"状态: {status}")

    # 3. 测试连接工具
    print("\n3. 测试连接工具...")
    connect_result = uav_tools.connect_drone.invoke("")
    print(f"连接结果: {connect_result}")

    # 4. 测试解锁
    print("\n4. 测试解锁...")
    arm_result = uav_tools.arm_drone.invoke("")
    print(f"解锁结果: {arm_result}")

    # 等待解锁完成
    await asyncio.sleep(3)

    # 5. 检查解锁后状态
    print("\n5. 检查解锁后状态...")
    status = uav_tools.get_drone_status.invoke("")
    print(f"状态: {status}")

    # 6. 测试起飞
    print("\n6. 测试起飞...")
    takeoff_result = uav_tools.takeoff_drone.invoke("")
    print(f"起飞结果: {takeoff_result}")

    # 等待起飞完成
    await asyncio.sleep(5)

    # 7. 检查起飞后状态
    print("\n7. 检查起飞后状态...")
    status = uav_tools.get_drone_status.invoke("")
    print(f"状态: {status}")

    # 8. 测试降落
    print("\n8. 测试降落...")
    land_result = uav_tools.land_drone.invoke("")
    print(f"降落结果: {land_result}")

    # 等待降落完成
    await asyncio.sleep(5)

    # 9. 测试锁定
    print("\n9. 测试锁定...")
    disarm_result = uav_tools.disarm_drone.invoke("")
    print(f"锁定结果: {disarm_result}")

    # 10. 最终状态检查
    print("\n10. 最终状态检查...")
    status = uav_tools.get_drone_status.invoke("")
    print(f"最终状态: {status}")
    
    print("\n=== UAV系统测试完成 ===")
    return True

async def main():
    """主函数"""
    try:
        await test_uav_system()
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())
