#!/usr/bin/env python3
"""
Simplified test script for UAV Agent LLM integration without ROS dependencies.
"""

import sys
import os

# Add the UAV agent scripts to the path
sys.path.append('src/uav_agent/scripts')

def test_llm_connection():
    """Test basic LLM connection."""
    print("🔍 Testing LLM connection...")
    try:
        from llm import get_llm
        
        llm = get_llm()
        response = llm.invoke("Hello! Can you respond with a simple greeting about being a UAV agent?")
        
        print("✅ LLM connection successful!")
        print(f"📝 Response: {response.content}")
        return True
    except Exception as e:
        print(f"❌ LLM connection failed: {e}")
        return False

def test_uav_tools_import():
    """Test UAV tools import."""
    print("\n🔍 Testing UAV tools import...")
    try:
        import tools.uav as uav_tools
        
        # Test that tools are available
        tools = [
            'connect_drone', 'arm_drone', 'disarm_drone', 
            'takeoff_drone', 'land_drone', 'move_drone',
            'set_manual_control', 'get_drone_status', 'emergency_stop'
        ]
        
        for tool_name in tools:
            if hasattr(uav_tools, tool_name):
                print(f"  ✅ {tool_name} - Available")
            else:
                print(f"  ❌ {tool_name} - Missing")
                return False
        
        print("✅ All UAV tools imported successfully!")
        return True
    except Exception as e:
        print(f"❌ UAV tools import failed: {e}")
        return False

def test_prompts():
    """Test prompts configuration."""
    print("\n🔍 Testing prompts configuration...")
    try:
        from prompts import get_prompts
        
        prompts = get_prompts()
        
        # Check that prompts have required attributes
        required_attrs = [
            'embodiment', 'about_your_operators',
            'critical_instructions', 'constraints_and_guardrails',
            'about_your_environment', 'about_your_capabilities'
        ]
        
        for attr in required_attrs:
            if hasattr(prompts, attr):
                print(f"  ✅ {attr} - Configured")
            else:
                print(f"  ❌ {attr} - Missing")
                return False
        
        print("✅ Prompts configuration successful!")
        return True
    except Exception as e:
        print(f"❌ Prompts configuration failed: {e}")
        return False

def test_tool_functionality():
    """Test individual tool functionality."""
    print("\n🔍 Testing tool functionality...")
    try:
        import tools.uav as uav_tools
        
        # Test get_drone_status (should work without actual drone)
        status_result = uav_tools.get_drone_status.invoke({})
        print(f"  ✅ get_drone_status: {status_result}")
        
        # Test connect_drone
        connect_result = uav_tools.connect_drone.invoke({})
        print(f"  ✅ connect_drone: {connect_result}")
        
        print("✅ Tool functionality test successful!")
        return True
    except Exception as e:
        print(f"❌ Tool functionality test failed: {e}")
        return False

def test_rosa_basic():
    """Test basic ROSA functionality without ROS."""
    print("\n🔍 Testing basic ROSA functionality...")
    try:
        from rosa import ROSA
        from llm import get_llm
        from prompts import get_prompts
        
        # Create a minimal ROSA instance
        llm = get_llm()
        prompts = get_prompts()
        
        # Create ROSA with minimal configuration
        rosa_agent = ROSA(
            ros_version=2,
            llm=llm,
            tools=[],  # Empty tools for basic test
            prompts=prompts,
            verbose=False,
            streaming=False
        )
        
        # Test a simple query
        response = rosa_agent.invoke("What are you?")
        
        print("✅ Basic ROSA functionality successful!")
        print(f"📝 Sample response: {response[:100]}...")
        return True
    except Exception as e:
        print(f"❌ Basic ROSA functionality failed: {e}")
        return False

def test_natural_language_processing():
    """Test natural language processing with UAV context."""
    print("\n🔍 Testing natural language processing...")
    try:
        from rosa import ROSA
        from llm import get_llm
        from prompts import get_prompts
        import tools.uav as uav_tools
        
        # Create ROSA with UAV tools
        llm = get_llm()
        prompts = get_prompts()
        
        rosa_agent = ROSA(
            ros_version=2,
            llm=llm,
            tool_packages=[uav_tools],
            prompts=prompts,
            verbose=False,
            streaming=False
        )
        
        # Test UAV-specific queries
        test_queries = [
            "What is your current status?",
            "How do I safely operate a drone?",
            "What tools do you have available?"
        ]
        
        for query in test_queries:
            print(f"  🔍 Testing query: '{query}'")
            response = rosa_agent.invoke(query)
            print(f"  ✅ Response length: {len(response)} characters")
        
        print("✅ Natural language processing test successful!")
        return True
    except Exception as e:
        print(f"❌ Natural language processing test failed: {e}")
        return False

def main():
    """Run all tests."""
    print("🚁 ROSA UAV Agent - Simplified LLM Integration Test")
    print("=" * 60)
    
    tests = [
        test_llm_connection,
        test_uav_tools_import,
        test_prompts,
        test_tool_functionality,
        test_rosa_basic,
        test_natural_language_processing,
    ]
    
    results = []
    for test in tests:
        result = test()
        results.append(result)
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary:")
    passed = sum(results)
    total = len(results)
    
    print(f"✅ Passed: {passed}/{total}")
    print(f"❌ Failed: {total - passed}/{total}")
    
    if passed == total:
        print("\n🎉 All tests passed! UAV Agent LLM integration is working!")
        print("💡 Note: ROS integration will be tested separately.")
        return True
    else:
        print("\n⚠️  Some tests failed. Please check the errors above.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
