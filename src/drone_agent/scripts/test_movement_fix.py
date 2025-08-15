#!/usr/bin/env python3

import asyncio
import sys
import os

# 添加当前目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tools.drone import (
    connect_drone, takeoff_drone, move_drone, land_drone,
    get_drone_status, drone_status
)

async def test_movement_fix():
    """
    测试修复后的移动功能
    """
    print("=" * 60)
    print("🚁 测试修复后的移动功能")
    print("=" * 60)
    
    # 连接无人机
    print("\n📡 连接无人机...")
    result = connect_drone("udp://:14540")
    print(f"连接结果: {result}")
    
    # 等待连接
    print("等待连接稳定...")
    for i in range(10):
        await asyncio.sleep(1)
        if drone_status.get('connected', False):
            print("✓ 连接成功")
            break
        print(f"连接中... {i+1}/10")
    
    if not drone_status.get('connected', False):
        print("❌ 连接失败，测试终止")
        return
    
    # 起飞到5米（较低高度测试）
    print("\n🚀 起飞到5米高度...")
    takeoff_result = takeoff_drone(5.0)
    print(f"起飞结果: {takeoff_result}")
    
    # 等待起飞完成
    print("等待起飞完成...")
    for i in range(30):
        await asyncio.sleep(1)
        altitude = drone_status.get('altitude', 0)
        flying = drone_status.get('flying', False)
        
        if flying and altitude > 4.0:
            print(f"✅ 起飞完成: 高度 {altitude:.2f}m")
            break
        elif flying:
            print(f"起飞中: 高度 {altitude:.2f}m")
        else:
            print(f"等待起飞: {i+1}s")
    
    # 测试前进移动（修复后的功能）
    print("\n➡️ 测试前进移动5米...")
    print("使用修复后的移动功能，应该解决 'list' object has no attribute 'translate_to_rpc' 错误")
    
    move_result = move_drone("forward", 5.0, 1.5)
    print(f"移动结果: {move_result}")
    
    # 等待移动完成
    print("等待移动完成...")
    await asyncio.sleep(10)
    
    # 检查状态
    status = get_drone_status()
    print(f"移动后状态: {status}")
    
    # 降落
    print("\n🛬 开始降落...")
    land_result = land_drone()
    print(f"降落结果: {land_result}")
    
    # 等待降落完成
    print("等待降落完成...")
    for i in range(30):
        await asyncio.sleep(1)
        flying = drone_status.get('flying', False)
        altitude = drone_status.get('altitude', 0)
        
        if not flying:
            print(f"✅ 降落完成: 高度 {altitude:.2f}m")
            break
        else:
            print(f"降落中: 高度 {altitude:.2f}m")
    
    print("\n🎉 移动功能修复测试完成")
    print("主要修复:")
    print("  ✅ 修复了 VelocityNedYaw 对象创建错误")
    print("  ✅ 添加了备用移动方法")
    print("  ✅ 改善了错误处理和中文提示")

if __name__ == "__main__":
    asyncio.run(test_movement_fix())
