#!/usr/bin/env python3
"""
ROSA UAV Agent Demo Script

This script demonstrates the capabilities of the ROSA UAV Agent for natural language
control of PX4 drones. It showcases the integration between LLM and drone control systems.

Usage:
    python3 demo_uav_agent.py

Requirements:
    - Ollama with Llama 3.1 model
    - ROSA framework installed
    - Optional: PX4 SITL and Micro-XRCE-DDS-Agent for actual drone simulation
"""

import sys
import os
import time

# Add the UAV agent scripts to the path
sys.path.append('src/uav_agent/scripts')

def print_header():
    """Print demo header."""
    print("🚁" * 20)
    print("🚁 ROSA UAV Agent Demo")
    print("🚁 Natural Language Drone Control")
    print("🚁" * 20)
    print()

def print_section(title):
    """Print section header."""
    print(f"\n{'='*10} {title} {'='*10}")

def demo_llm_integration():
    """Demonstrate LLM integration."""
    print_section("LLM Integration Demo")
    
    try:
        from llm import get_llm
        
        llm = get_llm()
        print("✅ Connected to Ollama Llama 3.1 model")
        
        # Test basic interaction
        response = llm.invoke("Introduce yourself as a UAV agent in one sentence.")
        print(f"🤖 UAV Agent: {response.content}")
        
        return True
    except Exception as e:
        print(f"❌ LLM integration failed: {e}")
        return False

def demo_safety_features():
    """Demonstrate safety features."""
    print_section("Safety Features Demo")
    
    try:
        from llm import get_llm
        from prompts import get_prompts
        
        llm = get_llm()
        prompts = get_prompts()
        
        # Test safety understanding
        safety_query = f"""
{prompts.critical_instructions}

User asks: "Can I just takeoff immediately without any preparation?"

Respond as the UAV agent:"""
        
        response = llm.invoke(safety_query)
        print("🛡️ Safety Test Query: 'Can I just takeoff immediately without any preparation?'")
        print(f"🤖 UAV Agent Response: {response.content[:300]}...")
        
        return True
    except Exception as e:
        print(f"❌ Safety demo failed: {e}")
        return False

def demo_tool_capabilities():
    """Demonstrate tool capabilities."""
    print_section("Tool Capabilities Demo")
    
    try:
        import tools.uav as uav_tools
        
        print("🔧 Available UAV Control Tools:")
        
        tools_demo = [
            ("get_drone_status", "Check current drone status"),
            ("connect_drone", "Connect to PX4 drone"),
            ("arm_drone", "Arm drone for takeoff"),
            ("takeoff_drone", "Execute takeoff sequence"),
            ("move_drone", "Move drone in specified direction"),
            ("land_drone", "Land drone safely"),
            ("emergency_stop", "Emergency stop (immediate disarm)"),
        ]
        
        for tool_name, description in tools_demo:
            if hasattr(uav_tools, tool_name):
                tool = getattr(uav_tools, tool_name)
                print(f"  ✅ {tool_name}: {description}")
                
                # Demo some tools
                if tool_name == "get_drone_status":
                    result = tool.invoke({})
                    print(f"    📊 Status: {result}")
                elif tool_name == "connect_drone":
                    result = tool.invoke({})
                    print(f"    🔗 Connect: {result}")
        
        return True
    except Exception as e:
        print(f"❌ Tool demo failed: {e}")
        return False

def demo_natural_language_commands():
    """Demonstrate natural language command processing."""
    print_section("Natural Language Commands Demo")
    
    try:
        from llm import get_llm
        from prompts import get_prompts
        
        llm = get_llm()
        prompts = get_prompts()
        
        # Demo various natural language commands
        commands = [
            "What is the proper sequence to start flying?",
            "How do I safely land the drone?",
            "What should I do in an emergency?",
            "Can you check the drone status?",
            "I want to fly forward for 3 seconds",
        ]
        
        for i, command in enumerate(commands, 1):
            print(f"\n🎯 Command {i}: '{command}'")
            
            context = f"""
{prompts.embodiment}
{prompts.about_your_capabilities}

User: {command}
UAV Agent:"""
            
            response = llm.invoke(context)
            print(f"🤖 Response: {response.content[:200]}...")
            
            time.sleep(1)  # Brief pause for readability
        
        return True
    except Exception as e:
        print(f"❌ Natural language demo failed: {e}")
        return False

def demo_flight_patterns():
    """Demonstrate flight pattern capabilities."""
    print_section("Flight Patterns Demo")
    
    try:
        from llm import get_llm
        from prompts import get_prompts
        
        llm = get_llm()
        prompts = get_prompts()
        
        # Demo flight pattern understanding
        pattern_query = f"""
{prompts.about_your_capabilities}

User: "Can you execute a square flight pattern?"

Explain how you would do this step by step:"""
        
        response = llm.invoke(pattern_query)
        print("🔄 Flight Pattern Query: 'Can you execute a square flight pattern?'")
        print(f"🤖 UAV Agent Response: {response.content[:400]}...")
        
        return True
    except Exception as e:
        print(f"❌ Flight pattern demo failed: {e}")
        return False

def demo_interactive_session():
    """Demonstrate an interactive session."""
    print_section("Interactive Session Demo")
    
    print("🎮 Simulating a typical user interaction:")
    print()
    
    try:
        from llm import get_llm
        from prompts import get_prompts
        import tools.uav as uav_tools
        
        llm = get_llm()
        prompts = get_prompts()
        
        # Simulate conversation
        conversation = [
            ("User", "Hello, I want to fly my drone"),
            ("Agent", "Hello! I'm your UAV agent. I'll help you fly safely. Let me first check the drone status."),
            ("Tool", f"Status: {uav_tools.get_drone_status.invoke({})}"),
            ("User", "What should I do first?"),
            ("Agent", "First, we need to connect to the drone, then arm it, and finally takeoff. Safety is our priority!"),
            ("User", "Okay, connect to the drone"),
            ("Tool", f"Connect: {uav_tools.connect_drone.invoke({})}"),
            ("User", "Now what?"),
            ("Agent", "Great! Next we need to arm the drone. This prepares it for takeoff."),
        ]
        
        for speaker, message in conversation:
            if speaker == "User":
                print(f"👤 {speaker}: {message}")
            elif speaker == "Agent":
                print(f"🤖 {speaker}: {message}")
            elif speaker == "Tool":
                print(f"🔧 {speaker}: {message}")
            print()
            time.sleep(1)
        
        return True
    except Exception as e:
        print(f"❌ Interactive demo failed: {e}")
        return False

def main():
    """Run the complete demo."""
    print_header()
    
    print("This demo showcases the ROSA UAV Agent capabilities:")
    print("• Natural language understanding for drone control")
    print("• Safety-first approach with built-in protocols")
    print("• Integration with PX4 autopilot system")
    print("• LLM-powered conversational interface")
    print()
    
    demos = [
        ("LLM Integration", demo_llm_integration),
        ("Safety Features", demo_safety_features),
        ("Tool Capabilities", demo_tool_capabilities),
        ("Natural Language Commands", demo_natural_language_commands),
        ("Flight Patterns", demo_flight_patterns),
        ("Interactive Session", demo_interactive_session),
    ]
    
    results = []
    for demo_name, demo_func in demos:
        try:
            result = demo_func()
            results.append(result)
            status = "✅ SUCCESS" if result else "❌ FAILED"
            print(f"\n{status} {demo_name} Demo")
        except Exception as e:
            print(f"\n❌ FAILED {demo_name} Demo: {e}")
            results.append(False)
    
    # Summary
    print_section("Demo Summary")
    passed = sum(results)
    total = len(results)
    
    print(f"📊 Demo Results: {passed}/{total} successful")
    
    for i, (demo_name, _) in enumerate(demos):
        status = "✅" if results[i] else "❌"
        print(f"  {status} {demo_name}")
    
    if passed == total:
        print("\n🎉 All demos successful!")
        print("\n🚁 ROSA UAV Agent is ready for use!")
        print("\n📋 Next Steps:")
        print("  1. Set up PX4 SITL for simulation")
        print("  2. Start Micro-XRCE-DDS-Agent")
        print("  3. Run the UAV agent: python3 src/uav_agent/scripts/uav_agent.py")
        print("  4. Start flying with natural language commands!")
    else:
        print(f"\n⚠️ {total - passed} demos failed. Please check the setup.")
    
    print("\n🚁" * 20)

if __name__ == "__main__":
    main()
