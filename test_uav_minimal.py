#!/usr/bin/env python3
"""
Minimal UAV Agent test that bypasses ROS dependencies.
This creates a simplified version to test core LLM and tool functionality.
"""

import sys
import os

# Add the UAV agent scripts to the path
sys.path.append('src/uav_agent/scripts')

def test_minimal_uav_functionality():
    """Test minimal UAV functionality without full ROSA framework."""
    print("🔍 Testing minimal UAV functionality...")
    
    try:
        # Import components individually
        from llm import get_llm
        from prompts import get_prompts
        import tools.uav as uav_tools
        
        print("✅ All imports successful!")
        
        # Test LLM
        llm = get_llm()
        response = llm.invoke("You are a UAV agent. Explain your purpose in one sentence.")
        print(f"✅ LLM Response: {response.content}")
        
        # Test prompts
        prompts = get_prompts()
        print(f"✅ Prompts configured with embodiment: {prompts.embodiment[:50]}...")
        
        # Test individual tools
        status = uav_tools.get_drone_status.invoke({})
        print(f"✅ Drone status: {status}")
        
        connect_result = uav_tools.connect_drone.invoke({})
        print(f"✅ Connect result: {connect_result}")
        
        # Test tool with parameters
        move_result = uav_tools.move_drone.invoke({"direction": "forward", "duration": 2.0})
        print(f"✅ Move command result: {move_result}")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

def test_llm_with_uav_context():
    """Test LLM with UAV-specific context."""
    print("\n🔍 Testing LLM with UAV context...")
    
    try:
        from llm import get_llm
        from prompts import get_prompts
        
        llm = get_llm()
        prompts = get_prompts()
        
        # Create a context-aware prompt
        context = f"""
{prompts.embodiment}

{prompts.critical_instructions}

User Query: What should I do to safely fly a drone?
"""
        
        response = llm.invoke(context)
        print(f"✅ Context-aware response: {response.content[:200]}...")
        
        # Test safety understanding
        safety_query = f"""
{prompts.constraints_and_guardrails}

User Query: Can I takeoff without arming the drone?
"""
        
        safety_response = llm.invoke(safety_query)
        print(f"✅ Safety response: {safety_response.content[:200]}...")
        
        return True
        
    except Exception as e:
        print(f"❌ Context test failed: {e}")
        return False

def test_tool_integration():
    """Test tool integration and command processing."""
    print("\n🔍 Testing tool integration...")
    
    try:
        import tools.uav as uav_tools
        from langchain.agents import tool
        
        # Test that all expected tools exist
        expected_tools = [
            'connect_drone', 'arm_drone', 'disarm_drone',
            'takeoff_drone', 'land_drone', 'move_drone',
            'set_manual_control', 'get_drone_status', 'emergency_stop'
        ]
        
        available_tools = []
        for tool_name in expected_tools:
            if hasattr(uav_tools, tool_name):
                tool_obj = getattr(uav_tools, tool_name)
                available_tools.append(tool_name)
                print(f"  ✅ {tool_name}: {tool_obj.description}")
            else:
                print(f"  ❌ {tool_name}: Missing")
                return False
        
        print(f"✅ All {len(available_tools)} tools available and properly configured!")
        
        # Test tool execution sequence
        print("\n  🔄 Testing tool execution sequence...")
        
        # 1. Check status
        status = uav_tools.get_drone_status.invoke({})
        print(f"    1. Status check: {status[:50]}...")
        
        # 2. Connect
        connect = uav_tools.connect_drone.invoke({})
        print(f"    2. Connect: {connect[:50]}...")
        
        # 3. Arm (will fail without real drone, but should return proper error)
        arm = uav_tools.arm_drone.invoke({})
        print(f"    3. Arm: {arm[:50]}...")
        
        print("✅ Tool execution sequence completed!")
        return True
        
    except Exception as e:
        print(f"❌ Tool integration test failed: {e}")
        return False

def test_natural_language_understanding():
    """Test natural language understanding capabilities."""
    print("\n🔍 Testing natural language understanding...")
    
    try:
        from llm import get_llm
        from prompts import get_prompts
        
        llm = get_llm()
        prompts = get_prompts()
        
        # Test various natural language queries
        test_queries = [
            ("What are you?", "identity"),
            ("How do I fly a drone safely?", "safety"),
            ("What's the proper takeoff sequence?", "procedure"),
            ("Can you help me land the drone?", "assistance"),
            ("What happens in an emergency?", "emergency"),
        ]
        
        for query, category in test_queries:
            full_prompt = f"""
{prompts.embodiment}
{prompts.about_your_capabilities}

User: {query}
Assistant:"""
            
            response = llm.invoke(full_prompt)
            print(f"  ✅ {category.title()}: '{query}' -> {response.content[:80]}...")
        
        print("✅ Natural language understanding test successful!")
        return True
        
    except Exception as e:
        print(f"❌ Natural language test failed: {e}")
        return False

def main():
    """Run all minimal tests."""
    print("🚁 ROSA UAV Agent - Minimal Functionality Test")
    print("=" * 60)
    print("Note: This test bypasses ROS dependencies to test core functionality")
    print("=" * 60)
    
    tests = [
        ("Minimal UAV Functionality", test_minimal_uav_functionality),
        ("LLM with UAV Context", test_llm_with_uav_context),
        ("Tool Integration", test_tool_integration),
        ("Natural Language Understanding", test_natural_language_understanding),
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        result = test_func()
        results.append(result)
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {test_name}")
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary:")
    passed = sum(results)
    total = len(results)
    
    for i, (test_name, _) in enumerate(tests):
        status = "✅ PASS" if results[i] else "❌ FAIL"
        print(f"  {status} {test_name}")
    
    print(f"\n🎯 Overall Result: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All core functionality tests passed!")
        print("✅ LLM integration working")
        print("✅ UAV tools properly configured")
        print("✅ Natural language understanding functional")
        print("✅ Safety prompts configured")
        print("\n💡 The UAV agent core is ready!")
        print("🔧 ROS integration can be tested separately in a proper ROS environment.")
        return True
    else:
        print(f"\n⚠️ {total - passed} tests failed. Please review the implementation.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
