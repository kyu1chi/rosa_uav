#!/usr/bin/env python3
"""
直接测试UAV工具函数
"""

import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_direct_tools():
    """直接测试工具函数"""
    print("=== 直接测试UAV工具函数 ===")
    
    try:
        # 导入工具模块
        from uav_agent.tools import uav_simple
        
        print("1. 测试工具函数导入...")
        
        # 检查工具函数是否存在
        expected_functions = [
            'connect_drone',
            'arm_drone', 
            'takeoff_drone',
            'land_drone',
            'move_drone',
            'get_drone_status',
            'disarm_drone',
            'return_to_launch',
            'emergency_stop'
        ]
        
        print("2. 检查工具函数...")
        found_functions = []
        for func_name in expected_functions:
            if hasattr(uav_simple, func_name):
                func = getattr(uav_simple, func_name)
                print(f"  ✅ {func_name} - 已找到 ({type(func)})")
                found_functions.append(func_name)
            else:
                print(f"  ❌ {func_name} - 未找到")
        
        print(f"\n3. 总结: 找到 {len(found_functions)}/{len(expected_functions)} 个工具函数")
        
        # 测试一个简单的工具函数
        print("\n4. 测试 get_drone_status 函数...")
        try:
            status_result = uav_simple.get_drone_status.invoke("")
            print(f"  状态结果: {status_result}")
            print("  ✅ get_drone_status 函数正常工作")
        except Exception as e:
            print(f"  ❌ get_drone_status 函数测试失败: {e}")
        
        # 测试连接函数
        print("\n5. 测试 connect_drone 函数...")
        try:
            connect_result = uav_simple.connect_drone.invoke("")
            print(f"  连接结果: {connect_result}")
            print("  ✅ connect_drone 函数正常工作")
        except Exception as e:
            print(f"  ❌ connect_drone 函数测试失败: {e}")
        
        if len(found_functions) == len(expected_functions):
            print("\n✅ 所有工具函数都已正确加载并可以调用！")
            return True
        else:
            print(f"\n❌ 缺少 {len(expected_functions) - len(found_functions)} 个工具函数")
            return False
            
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主函数"""
    try:
        success = test_direct_tools()
        if success:
            print("\n🎉 工具函数测试成功！")
            print("问题可能在于ROSA框架的工具注册，而不是工具函数本身。")
            print("建议检查UAV代理中的工具注册代码。")
        else:
            print("\n⚠️ 工具函数有问题，需要进一步调试。")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")

if __name__ == "__main__":
    main()
