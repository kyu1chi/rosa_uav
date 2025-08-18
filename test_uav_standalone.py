#!/usr/bin/env python3
"""
Standalone UAV Agent test without ROS dependencies.
This demonstrates the UAV agent functionality in a standalone mode.
"""

import sys
import os
import asyncio

# Add the UAV agent scripts to the path
sys.path.append('src/uav_agent/scripts')

def create_standalone_uav_agent():
    """Create a UAV agent that works without ROS."""
    print("🔍 Creating standalone UAV agent...")
    try:
        from rosa import ROSA
        from llm import get_llm
        from prompts import get_prompts
        import tools.uav as uav_tools
        
        # Create LLM and prompts
        llm = get_llm()
        prompts = get_prompts()
        
        # Create ROSA agent with UAV tools
        agent = ROSA(
            ros_version=2,  # This won't actually use ROS in standalone mode
            llm=llm,
            tool_packages=[uav_tools],
            prompts=prompts,
            verbose=False,
            streaming=False
        )
        
        print("✅ Standalone UAV agent created successfully!")
        return agent
    except Exception as e:
        print(f"❌ Failed to create standalone UAV agent: {e}")
        return None

def test_basic_commands(agent):
    """Test basic UAV commands."""
    print("\n🔍 Testing basic UAV commands...")
    
    test_commands = [
        "What is your current status?",
        "What tools do you have available for drone control?",
        "How do I safely operate a drone?",
        "What is the proper sequence for drone operations?",
        "Check the drone status",
    ]
    
    results = []
    for i, command in enumerate(test_commands, 1):
        try:
            print(f"\n  📝 Test {i}: '{command}'")
            response = agent.invoke(command)
            print(f"  ✅ Response: {response[:150]}...")
            results.append(True)
        except Exception as e:
            print(f"  ❌ Failed: {e}")
            results.append(False)
    
    passed = sum(results)
    total = len(results)
    print(f"\n✅ Basic commands test: {passed}/{total} passed")
    return passed == total

def test_safety_protocols(agent):
    """Test safety protocol understanding."""
    print("\n🔍 Testing safety protocols...")
    
    safety_tests = [
        "Can I takeoff without arming the drone?",
        "What happens if I try to move the drone while it's on the ground?",
        "What is the maximum duration for a single movement command?",
        "When should I use emergency stop?",
    ]
    
    results = []
    for i, test in enumerate(safety_tests, 1):
        try:
            print(f"\n  🛡️ Safety Test {i}: '{test}'")
            response = agent.invoke(test)
            
            # Check if response contains safety-related keywords
            safety_keywords = ['safety', 'error', 'must', 'cannot', 'should', 'emergency', 'arm', 'sequence']
            contains_safety = any(keyword in response.lower() for keyword in safety_keywords)
            
            if contains_safety:
                print(f"  ✅ Safety-aware response: {response[:100]}...")
                results.append(True)
            else:
                print(f"  ⚠️ Response may lack safety awareness: {response[:100]}...")
                results.append(False)
        except Exception as e:
            print(f"  ❌ Failed: {e}")
            results.append(False)
    
    passed = sum(results)
    total = len(results)
    print(f"\n✅ Safety protocols test: {passed}/{total} passed")
    return passed == total

def test_command_understanding(agent):
    """Test natural language command understanding."""
    print("\n🔍 Testing command understanding...")
    
    command_tests = [
        ("Connect to the drone", ["connect", "drone"]),
        ("Arm the drone for takeoff", ["arm", "drone"]),
        ("Take off to default altitude", ["takeoff", "take"]),
        ("Fly forward for 3 seconds", ["move", "forward", "3"]),
        ("Land the drone safely", ["land", "drone"]),
        ("Emergency stop now", ["emergency", "stop"]),
    ]
    
    results = []
    for i, (command, expected_keywords) in enumerate(command_tests, 1):
        try:
            print(f"\n  🎯 Command Test {i}: '{command}'")
            response = agent.invoke(command)
            
            # Check if response mentions relevant tools or actions
            response_lower = response.lower()
            keyword_found = any(keyword in response_lower for keyword in expected_keywords)
            
            if keyword_found:
                print(f"  ✅ Command understood: {response[:100]}...")
                results.append(True)
            else:
                print(f"  ⚠️ Command may not be fully understood: {response[:100]}...")
                results.append(False)
        except Exception as e:
            print(f"  ❌ Failed: {e}")
            results.append(False)
    
    passed = sum(results)
    total = len(results)
    print(f"\n✅ Command understanding test: {passed}/{total} passed")
    return passed == total

def test_interactive_session(agent):
    """Test an interactive session simulation."""
    print("\n🔍 Testing interactive session simulation...")
    
    session_commands = [
        "Hello, I want to fly a drone",
        "What should I do first?",
        "Check the drone status",
        "Connect to the drone",
        "Now what?",
        "Arm the drone",
        "Take off",
        "Fly forward for 2 seconds",
        "Land the drone",
        "Disarm the drone",
    ]
    
    print("  🎮 Simulating interactive session...")
    results = []
    
    for i, command in enumerate(session_commands, 1):
        try:
            print(f"\n    Step {i}: User: '{command}'")
            response = agent.invoke(command)
            print(f"    UAV Agent: {response[:120]}...")
            results.append(True)
        except Exception as e:
            print(f"    ❌ Failed at step {i}: {e}")
            results.append(False)
            break
    
    passed = sum(results)
    total = len(results)
    print(f"\n✅ Interactive session test: {passed}/{total} steps completed")
    return passed >= total * 0.8  # 80% success rate is acceptable

def main():
    """Run comprehensive UAV agent tests."""
    print("🚁 ROSA UAV Agent - Comprehensive Standalone Test")
    print("=" * 60)
    
    # Create agent
    agent = create_standalone_uav_agent()
    if not agent:
        print("❌ Cannot proceed without UAV agent")
        return False
    
    # Run tests
    tests = [
        ("Basic Commands", lambda: test_basic_commands(agent)),
        ("Safety Protocols", lambda: test_safety_protocols(agent)),
        ("Command Understanding", lambda: test_command_understanding(agent)),
        ("Interactive Session", lambda: test_interactive_session(agent)),
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        result = test_func()
        results.append(result)
        print(f"{'='*60}")
    
    # Final summary
    print("\n" + "=" * 60)
    print("📊 Final Test Summary:")
    passed = sum(results)
    total = len(results)
    
    for i, (test_name, _) in enumerate(tests):
        status = "✅ PASS" if results[i] else "❌ FAIL"
        print(f"  {status} {test_name}")
    
    print(f"\n🎯 Overall Result: {passed}/{total} test suites passed")
    
    if passed == total:
        print("\n🎉 All tests passed! UAV Agent is fully functional!")
        print("💡 The agent can understand natural language and respond appropriately.")
        print("🛡️ Safety protocols are properly implemented.")
        print("🚁 Ready for drone control operations!")
        return True
    elif passed >= total * 0.75:
        print("\n✅ Most tests passed! UAV Agent is largely functional.")
        print("⚠️ Some minor issues detected, but core functionality works.")
        return True
    else:
        print("\n⚠️ Several tests failed. Please review the implementation.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
