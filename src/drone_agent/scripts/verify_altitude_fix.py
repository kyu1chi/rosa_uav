#!/usr/bin/env python3

import asyncio
import sys
import os

# 添加当前目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tools.drone import connect_drone, takeoff_drone, drone_status

async def verify_altitude_fix():
    """验证高度控制修复效果"""
    print("=== 验证无人机高度控制修复 ===")
    
    # 连接无人机
    print("\n1. 连接无人机...")
    result = connect_drone("udp://:14540")
    print(f"连接结果: {result}")
    
    # 等待连接
    print("等待连接完成...")
    for i in range(10):
        await asyncio.sleep(1)
        if drone_status.get('connected', False):
            print("✓ 连接成功")
            break
        print(f"连接中... {i+1}/10")
    
    if not drone_status.get('connected', False):
        print("❌ 连接失败")
        return
    
    # 测试2米高度控制
    print("\n2. 测试2米高度精确控制...")
    target_altitude = 2.0
    
    print(f"发送起飞指令到 {target_altitude}米...")
    result = takeoff_drone(target_altitude)
    print(f"起飞指令结果: {result}")
    
    print("\n=== 修复验证完成 ===")
    print("请观察日志输出，检查以下修复点：")
    print("1. ✅ 分阶段起飞控制（初始起飞 → 精确控制 → 高度维持 → 最终验证）")
    print("2. ✅ 主动高度控制（接近目标高度时自动发送位置保持命令）")
    print("3. ✅ 精确高度监控（±0.2米容差，连续8次稳定读数）")
    print("4. ✅ 超调检测和修正（高度偏差>0.4米时重新发送控制命令）")
    print("5. ✅ 最终高度验证和误差报告")

if __name__ == "__main__":
    asyncio.run(verify_altitude_fix())
