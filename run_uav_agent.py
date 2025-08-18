#!/usr/bin/env python3
"""
Standalone UAV Agent Runner

This script runs the ROSA UAV Agent without ROS dependencies.
It's designed to work in environments where ROS2 is not properly configured.

Usage:
    python3 run_uav_agent.py

Requirements:
    - Ollama with Llama 3.1 model
    - ROSA framework installed
    - MAVSDK for drone communication
"""

import sys
import os
import asyncio

# Add the UAV agent to the path
sys.path.insert(0, 'src/uav_agent/uav_agent')

def create_standalone_rosa_agent():
    """Create a ROSA agent without ROS dependencies."""
    try:
        from rosa import ROSA
        from llm import get_llm
        from prompts import get_prompts
        import tools.uav as uav_tools
        from langchain.agents import tool, Tool
        
        # Create additional tools
        @tool
        def flight_pattern_square(side_length: float = 2.0):
            """Execute a square flight pattern."""
            if side_length <= 0 or side_length > 5:
                return "Error: Side length must be between 0 and 5 seconds for safety."
            
            return f"""
            Executing square flight pattern with {side_length} second sides.
            
            <ROSA_INSTRUCTIONS>
                Execute the following sequence:
                1. Move forward for {side_length} seconds
                2. Move right for {side_length} seconds  
                3. Move backward for {side_length} seconds
                4. Move left for {side_length} seconds
                5. Return to hover position
            </ROSA_INSTRUCTIONS>
            """

        def hover_in_place(input: str = ""):
            """Make the drone hover in place with neutral controls."""
            return """
            Setting drone to hover in place.
            
            <ROSA_INSTRUCTIONS>
                Use set_manual_control with roll=0, pitch=0, throttle=0.5, yaw=0 to maintain hover position.
            </ROSA_INSTRUCTIONS>
            """

        hover_tool = Tool(
            name="hover_in_place",
            func=hover_in_place,
            description="Make the drone hover in place (neutral position).",
        )

        # Get LLM and prompts
        llm = get_llm()
        prompts = get_prompts()
        
        # Create ROSA agent with minimal ROS version (won't actually use ROS tools)
        agent = ROSA(
            ros_version=2,
            llm=llm,
            tools=[flight_pattern_square, hover_tool],
            tool_packages=[uav_tools],
            prompts=prompts,
            verbose=False,
            streaming=True,
            blacklist=["ros2", "node", "topic", "service", "param"]  # Blacklist all ROS tools
        )
        
        return agent
    except Exception as e:
        print(f"Error creating ROSA agent: {e}")
        return None

async def run_interactive_session(agent):
    """Run an interactive session with the UAV agent."""
    from rich.console import Console
    from rich.text import Text
    from rich.panel import Panel
    from rich.markdown import Markdown
    import pyinputplus as pyip
    
    console = Console()
    
    # Print greeting
    greeting = Text(
        "\nHi! I'm the ROSA-UAV agent 🚁🤖. I can help you control PX4 drones safely using natural language!\n"
    )
    greeting.stylize("frame bold blue")
    greeting.append(
        "Try 'help', 'examples', 'clear' or 'exit'. ",
        style="italic",
    )
    greeting.append(
        "Remember: Safety first! Always follow proper flight procedures.",
        style="bold red",
    )
    
    examples = [
        "Connect to the drone and check status",
        "Arm the drone and take off",
        "Fly forward for 3 seconds then hover",
        "Execute a square flight pattern",
        "Move up 2 seconds, then move down 2 seconds",
        "Land the drone and disarm",
        "Check current drone status",
        "Emergency stop the drone",
    ]
    
    def get_help():
        return """
# ROSA UAV Agent Help

## Basic Commands
- "Connect to the drone"
- "Arm the drone"
- "Take off"
- "Fly forward for 2 seconds"
- "Land the drone"
- "Check drone status"

## Special Commands
- `help` - Show this help
- `examples` - Show example commands
- `clear` - Clear chat history
- `exit` - Exit the application

## Safety Notes
- Always follow the sequence: Connect → Arm → Takeoff → Fly → Land → Disarm
- All movements are limited to 10 seconds maximum
- The system prevents unsafe operations
"""

    def choose_example():
        return pyip.inputMenu(
            examples,
            prompt="\nChoose an example and press enter: \n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1",
        )

    # Initialize UAV system
    console.print("[yellow]Initializing UAV system...[/yellow]")
    try:
        import tools.uav as uav_tools
        await uav_tools.initialize_uav_system()
        console.print("[green]UAV system initialized successfully![/green]")
    except Exception as e:
        console.print(f"[red]Failed to initialize UAV system: {e}[/red]")
        console.print("[yellow]You can still use the agent, but drone commands may fail.[/yellow]")

    # Main interaction loop
    while True:
        console.print(greeting)
        
        try:
            user_input = pyip.inputStr("> ", default="help")
            
            if user_input == "exit":
                console.print("[yellow]Shutting down UAV agent. Fly safe! ✈️[/yellow]")
                break
            elif user_input == "help":
                console.print(Panel(Markdown(get_help()), title="Help", border_style="blue"))
            elif user_input == "examples":
                example = choose_example()
                console.print(f"[green]Selected example: {example}[/green]")
                # Process the example
                response = agent.invoke(example)
                console.print(Panel(Markdown(response), title="UAV Response", border_style="green"))
            elif user_input == "clear":
                agent.clear_chat()
                os.system("clear")
            else:
                # Process user query
                console.print("[blue]Processing...[/blue]")
                response = agent.invoke(user_input)
                console.print(Panel(Markdown(response), title="UAV Response", border_style="green"))
                
        except KeyboardInterrupt:
            console.print("\n[yellow]UAV Agent interrupted. Goodbye! ✈️[/yellow]")
            break
        except Exception as e:
            console.print(f"[red]Error: {e}[/red]")

async def main():
    """Main function."""
    print("🚁 ROSA UAV Agent - Standalone Mode")
    print("=" * 50)
    
    # Create agent
    print("Creating UAV agent...")
    agent = create_standalone_rosa_agent()
    
    if not agent:
        print("❌ Failed to create UAV agent. Please check your setup.")
        return
    
    print("✅ UAV agent created successfully!")
    print("🚀 Starting interactive session...")
    print()
    
    # Run interactive session
    await run_interactive_session(agent)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Error: {e}")
