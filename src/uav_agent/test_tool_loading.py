#!/usr/bin/env python3
"""
测试工具是否正确加载到UAV代理中
"""

import sys
import os

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_tool_loading():
    """测试工具加载"""
    print("=== 测试UAV代理工具加载 ===")
    
    try:
        # 导入UAV代理
        from uav_agent.uav_agent import UAVAgent
        
        print("1. 创建UAV代理实例...")
        agent = UAVAgent(verbose=True, streaming=False)
        
        print("2. 检查可用工具...")
        
        # 获取工具列表
        if hasattr(agent, '_UAVAgent__tools') or hasattr(agent, 'tools'):
            tools = getattr(agent, '_UAVAgent__tools', None) or getattr(agent, 'tools', None)
            if tools:
                print(f"发现 {len(tools)} 个工具:")
                for i, tool in enumerate(tools, 1):
                    tool_name = getattr(tool, 'name', str(tool))
                    tool_desc = getattr(tool, 'description', 'No description')
                    print(f"  {i}. {tool_name}: {tool_desc}")
            else:
                print("❌ 没有找到工具列表")
        else:
            print("❌ 代理没有工具属性")
            
        # 检查特定的UAV工具
        expected_tools = [
            'connect_drone',
            'arm_drone', 
            'takeoff_drone',
            'land_drone',
            'move_drone',
            'get_drone_status'
        ]
        
        print("\n3. 检查关键UAV工具...")
        found_tools = []
        if hasattr(agent, '_UAVAgent__tools') or hasattr(agent, 'tools'):
            tools = getattr(agent, '_UAVAgent__tools', None) or getattr(agent, 'tools', None)
            if tools:
                tool_names = [getattr(tool, 'name', str(tool)) for tool in tools]
                for expected_tool in expected_tools:
                    if expected_tool in tool_names:
                        print(f"  ✅ {expected_tool} - 已加载")
                        found_tools.append(expected_tool)
                    else:
                        print(f"  ❌ {expected_tool} - 未找到")
        
        print(f"\n4. 总结: 找到 {len(found_tools)}/{len(expected_tools)} 个关键工具")
        
        if len(found_tools) == len(expected_tools):
            print("✅ 所有关键工具都已正确加载！")
            return True
        else:
            print("❌ 部分工具缺失，可能影响功能")
            return False
            
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主函数"""
    try:
        success = test_tool_loading()
        if success:
            print("\n🎉 工具加载测试成功！UAV代理应该能正确响应命令。")
        else:
            print("\n⚠️ 工具加载有问题，需要进一步调试。")
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")

if __name__ == "__main__":
    main()
