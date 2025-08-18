#!/usr/bin/env python3
"""
Test script for UAV Agent LLM integration.
This script tests the LLM connectivity and basic UAV agent functionality.
"""

import sys
import os
import asyncio

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

def test_rosa_integration():
    """Test ROSA framework integration."""
    print("\n🔍 Testing ROSA integration...")
    try:
        from rosa import ROSA
        from llm import get_llm
        from prompts import get_prompts
        import tools.uav as uav_tools
        
        # Create a minimal ROSA instance
        llm = get_llm()
        prompts = get_prompts()
        
        rosa_agent = ROSA(
            ros_version=2,
            llm=llm,
            tools=[],  # Empty tools for basic test
            tool_packages=[uav_tools],
            prompts=prompts,
            verbose=False,
            streaming=False
        )
        
        # Test a simple query
        response = rosa_agent.invoke("What are you?")
        
        print("✅ ROSA integration successful!")
        print(f"📝 Sample response: {response[:100]}...")
        return True
    except Exception as e:
        print(f"❌ ROSA integration failed: {e}")
        return False

async def test_uav_agent_basic():
    """Test basic UAV agent functionality."""
    print("\n🔍 Testing UAV Agent basic functionality...")
    try:
        from uav_agent import UAVAgent
        
        # Create UAV agent instance
        agent = UAVAgent(streaming=False, verbose=False)
        
        # Test a simple query
        response = agent.invoke("What is your purpose?")
        
        print("✅ UAV Agent basic functionality successful!")
        print(f"📝 Sample response: {response[:100]}...")
        return True
    except Exception as e:
        print(f"❌ UAV Agent basic functionality failed: {e}")
        return False

def main():
    """Run all tests."""
    print("🚁 ROSA UAV Agent - LLM Integration Test")
    print("=" * 50)
    
    tests = [
        test_llm_connection,
        test_uav_tools_import,
        test_prompts,
        test_rosa_integration,
    ]
    
    results = []
    for test in tests:
        result = test()
        results.append(result)
    
    # Test UAV agent (async)
    print("\n🔍 Testing UAV Agent basic functionality...")
    try:
        result = asyncio.run(test_uav_agent_basic())
        results.append(result)
    except Exception as e:
        print(f"❌ UAV Agent test failed: {e}")
        results.append(False)
    
    # Summary
    print("\n" + "=" * 50)
    print("📊 Test Summary:")
    passed = sum(results)
    total = len(results)
    
    print(f"✅ Passed: {passed}/{total}")
    print(f"❌ Failed: {total - passed}/{total}")
    
    if passed == total:
        print("\n🎉 All tests passed! UAV Agent is ready for use.")
        return True
    else:
        print("\n⚠️  Some tests failed. Please check the errors above.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
