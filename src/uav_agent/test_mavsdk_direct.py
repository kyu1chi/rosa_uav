#!/usr/bin/env python3
"""
直接测试MAVSDK连接的脚本
"""

import asyncio
from mavsdk import System

async def test_direct_connection():
    """直接测试MAVSDK连接"""
    print("=== 直接MAVSDK连接测试 ===")
    
    drone = System()
    
    try:
        # 连接到无人机
        print("1. 连接到无人机...")
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        
        print("2. 等待连接...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("✅ 连接成功!")
                break
        
        # 检查健康状态
        print("3. 检查健康状态...")
        async for health in drone.telemetry.health():
            print(f"全球定位: {health.is_global_position_ok}")
            print(f"本地定位: {health.is_local_position_ok}")
            print(f"家位置: {health.is_home_position_ok}")
            print(f"陀螺仪: {health.is_gyrometer_calibration_ok}")
            print(f"加速度计: {health.is_accelerometer_calibration_ok}")
            print(f"磁力计: {health.is_magnetometer_calibration_ok}")
            break
        
        # 检查飞行模式
        print("4. 检查飞行模式...")
        async for flight_mode in drone.telemetry.flight_mode():
            print(f"当前飞行模式: {flight_mode}")
            break
        
        # 检查解锁状态
        print("5. 检查解锁状态...")
        async for armed in drone.telemetry.armed():
            print(f"解锁状态: {armed}")
            break
        
        # 尝试解锁
        print("6. 尝试解锁...")
        try:
            await drone.action.arm()
            print("✅ 解锁成功!")
            
            # 等待一下
            await asyncio.sleep(2)
            
            # 检查解锁状态
            async for armed in drone.telemetry.armed():
                print(f"解锁后状态: {armed}")
                break
            
            # 尝试起飞
            print("7. 尝试起飞...")
            await drone.action.takeoff()
            print("✅ 起飞命令发送成功!")
            
            # 等待起飞
            print("8. 等待起飞完成...")
            await asyncio.sleep(10)
            
            # 检查位置
            async for position in drone.telemetry.position():
                print(f"当前位置 - 纬度: {position.latitude_deg}, 经度: {position.longitude_deg}, 高度: {position.absolute_altitude_m}")
                break
            
            # 降落
            print("9. 降落...")
            await drone.action.land()
            print("✅ 降落命令发送成功!")
            
            # 等待降落
            await asyncio.sleep(10)
            
            # 锁定
            print("10. 锁定...")
            await drone.action.disarm()
            print("✅ 锁定成功!")
            
        except Exception as e:
            print(f"❌ 操作失败: {e}")
        
    except Exception as e:
        print(f"❌ 连接失败: {e}")
    
    print("=== 测试完成 ===")

if __name__ == "__main__":
    asyncio.run(test_direct_connection())
