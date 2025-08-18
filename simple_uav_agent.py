#!/usr/bin/env python3
"""
Simple UAV Agent - No ROS Dependencies

This is a simplified version of the UAV agent that works without ROS.
It demonstrates the core functionality using direct LLM integration.

Usage:
    python3 simple_uav_agent.py

Requirements:
    - Ollama with Llama 3.1 model
    - MAVSDK for drone communication
"""

import sys
import os
import asyncio

# Add the UAV agent to the path
sys.path.insert(0, 'src/uav_agent/uav_agent')

class SimpleUAVAgent:
    """Simplified UAV Agent without ROS dependencies."""
    
    def __init__(self):
        self.chat_history = []
        self.llm = None
        self.prompts = None
        self.tools = {}
        self._initialize()
    
    def _initialize(self):
        """Initialize the agent components."""
        try:
            # Import components
            from llm import get_llm
            from prompts import get_prompts
            import tools.uav as uav_tools
            
            # Initialize LLM and prompts
            self.llm = get_llm()
            self.prompts = get_prompts()
            
            # Register tools
            self.tools = {
                'connect_drone': uav_tools.connect_drone,
                'arm_drone': uav_tools.arm_drone,
                'disarm_drone': uav_tools.disarm_drone,
                'takeoff_drone': uav_tools.takeoff_drone,
                'land_drone': uav_tools.land_drone,
                'move_drone': uav_tools.move_drone,
                'set_manual_control': uav_tools.set_manual_control,
                'get_drone_status': uav_tools.get_drone_status,
                'emergency_stop': uav_tools.emergency_stop,
            }
            
            print("✅ UAV Agent initialized successfully!")
            
        except Exception as e:
            print(f"❌ Failed to initialize UAV Agent: {e}")
            raise
    
    def _create_context(self, user_input):
        """Create context for the LLM."""
        context = f"""
{self.prompts.embodiment}

{self.prompts.critical_instructions}

{self.prompts.about_your_capabilities}

Available Tools:
"""
        for tool_name, tool in self.tools.items():
            context += f"- {tool_name}: {tool.description}\n"
        
        context += f"""
Chat History:
"""
        for message in self.chat_history[-6:]:  # Last 6 messages for context
            context += f"{message}\n"
        
        context += f"""
User: {user_input}

As the UAV agent, respond helpfully and safely. If the user wants to perform a drone operation, explain what you would do and mention the specific tool you would use. Always prioritize safety.

UAV Agent:"""
        
        return context
    
    def _extract_tool_calls(self, response):
        """Extract tool calls from the response (simplified)."""
        tool_calls = []
        
        # Simple pattern matching for tool calls
        for tool_name in self.tools.keys():
            if tool_name in response.lower():
                tool_calls.append(tool_name)
        
        return tool_calls
    
    def _execute_tool(self, tool_name, params=None):
        """Execute a tool and return the result."""
        if tool_name not in self.tools:
            return f"Error: Tool '{tool_name}' not found."
        
        try:
            tool = self.tools[tool_name]
            if params:
                result = tool.invoke(params)
            else:
                result = tool.invoke({})
            return result
        except Exception as e:
            return f"Error executing {tool_name}: {e}"
    
    def process_query(self, user_input):
        """Process a user query and return a response."""
        try:
            # Create context
            context = self._create_context(user_input)
            
            # Get LLM response
            response = self.llm.invoke(context)
            llm_response = response.content
            
            # Check for specific tool requests
            tool_results = []
            
            # Simple command processing
            user_lower = user_input.lower()
            
            if "status" in user_lower or "check" in user_lower:
                result = self._execute_tool("get_drone_status")
                tool_results.append(f"🔧 Status Check: {result}")
            
            elif "connect" in user_lower:
                result = self._execute_tool("connect_drone")
                tool_results.append(f"🔗 Connect: {result}")
            
            elif "arm" in user_lower and "disarm" not in user_lower:
                result = self._execute_tool("arm_drone")
                tool_results.append(f"🔓 Arm: {result}")
            
            elif "disarm" in user_lower:
                result = self._execute_tool("disarm_drone")
                tool_results.append(f"🔒 Disarm: {result}")
            
            elif "takeoff" in user_lower or "take off" in user_lower:
                result = self._execute_tool("takeoff_drone")
                tool_results.append(f"🚀 Takeoff: {result}")
            
            elif "land" in user_lower:
                result = self._execute_tool("land_drone")
                tool_results.append(f"🛬 Land: {result}")
            
            elif "emergency" in user_lower:
                result = self._execute_tool("emergency_stop")
                tool_results.append(f"🚨 Emergency Stop: {result}")
            
            elif any(direction in user_lower for direction in ["forward", "backward", "left", "right", "up", "down"]):
                # Extract direction and duration
                direction = None
                duration = 1.0
                
                for d in ["forward", "backward", "left", "right", "up", "down"]:
                    if d in user_lower:
                        direction = d
                        break
                
                # Try to extract duration
                import re
                duration_match = re.search(r'(\d+(?:\.\d+)?)\s*(?:second|sec)', user_lower)
                if duration_match:
                    duration = float(duration_match.group(1))
                
                if direction:
                    result = self._execute_tool("move_drone", {"direction": direction, "duration": duration})
                    tool_results.append(f"🎯 Move {direction}: {result}")
            
            # Combine LLM response with tool results
            final_response = llm_response
            if tool_results:
                final_response += "\n\n**Tool Execution Results:**\n" + "\n".join(tool_results)
            
            # Update chat history
            self.chat_history.append(f"User: {user_input}")
            self.chat_history.append(f"UAV Agent: {final_response}")
            
            return final_response
            
        except Exception as e:
            return f"Error processing query: {e}"
    
    def clear_history(self):
        """Clear chat history."""
        self.chat_history = []

async def run_interactive_session():
    """Run an interactive session."""
    from rich.console import Console
    from rich.text import Text
    from rich.panel import Panel
    from rich.markdown import Markdown
    import pyinputplus as pyip
    
    console = Console()
    
    # Create agent
    console.print("[yellow]Initializing Simple UAV Agent...[/yellow]")
    try:
        agent = SimpleUAVAgent()
    except Exception as e:
        console.print(f"[red]Failed to create agent: {e}[/red]")
        return
    
    # Initialize UAV system
    console.print("[yellow]Initializing UAV system...[/yellow]")
    try:
        import tools.uav as uav_tools
        await uav_tools.initialize_uav_system()
        console.print("[green]UAV system initialized![/green]")
    except Exception as e:
        console.print(f"[red]UAV system initialization failed: {e}[/red]")
        console.print("[yellow]Continuing without UAV system...[/yellow]")
    
    # Greeting
    greeting = Text(
        "\n🚁 Simple UAV Agent - Ready for Commands! 🤖\n"
    )
    greeting.stylize("frame bold blue")
    greeting.append(
        "Try: 'check status', 'connect drone', 'help', or 'exit'\n",
        style="italic",
    )
    greeting.append(
        "Safety First! Always follow proper procedures.",
        style="bold red",
    )
    
    examples = [
        "Check drone status",
        "Connect to the drone",
        "Arm the drone",
        "Take off",
        "Fly forward for 3 seconds",
        "Land the drone",
        "Emergency stop",
    ]
    
    # Main loop
    while True:
        console.print(greeting)
        
        try:
            user_input = pyip.inputStr("> ", default="help")
            
            if user_input.lower() == "exit":
                console.print("[yellow]Goodbye! Fly safe! ✈️[/yellow]")
                break
            elif user_input.lower() == "help":
                help_text = """
# Simple UAV Agent Commands

## Basic Commands:
- "check status" - Check drone status
- "connect drone" - Connect to drone
- "arm drone" - Arm for takeoff
- "take off" - Execute takeoff
- "fly [direction] for [X] seconds" - Move drone
- "land drone" - Land safely
- "emergency stop" - Emergency stop

## Special Commands:
- "help" - Show this help
- "clear" - Clear chat history
- "exit" - Exit application

## Example:
"fly forward for 3 seconds"
"""
                console.print(Panel(Markdown(help_text), title="Help", border_style="blue"))
            elif user_input.lower() == "clear":
                agent.clear_history()
                os.system("clear")
                console.print("[green]Chat history cleared![/green]")
            else:
                # Process query
                console.print("[blue]Processing...[/blue]")
                response = agent.process_query(user_input)
                console.print(Panel(Markdown(response), title="UAV Agent Response", border_style="green"))
                
        except KeyboardInterrupt:
            console.print("\n[yellow]Interrupted. Goodbye! ✈️[/yellow]")
            break
        except Exception as e:
            console.print(f"[red]Error: {e}[/red]")

def main():
    """Main function."""
    print("🚁 Simple UAV Agent - No ROS Dependencies")
    print("=" * 50)
    
    try:
        asyncio.run(run_interactive_session())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
