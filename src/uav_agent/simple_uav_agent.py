#!/usr/bin/env python3
"""
简化的UAV代理，绕过ROS环境问题
"""

import asyncio
import os
from datetime import datetime

import dotenv
import pyinputplus as pyip
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain.prompts import ChatPromptTemplate
from rich.console import Console
from rich.console import Group
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text

# 导入UAV工具
from uav_agent.tools import uav_simple as uav_tools
from uav_agent.tools import uav_ros2_move as ros2_tools
from uav_agent.tools import uav_offboard as offboard_tools
from uav_agent.llm import get_llm

class SimpleUAVAgent:
    """简化的UAV代理"""
    
    def __init__(self, streaming: bool = True):
        self.__llm = get_llm()
        self.__streaming = streaming
        
        # 创建工具列表
        self.tools = [
            uav_tools.connect_drone,
            uav_tools.arm_drone,
            uav_tools.disarm_drone,
            uav_tools.takeoff_drone,
            uav_tools.takeoff_drone_to_altitude,
            uav_tools.land_drone,
            uav_tools.return_to_launch,
            uav_tools.move_drone,
            # uav_tools.move_drone_distance,  # 移除有问题的函数
            ros2_tools.move_drone_ros2,  # ROS2移动方法
            offboard_tools.move_drone_offboard,  # MAVSDK offboard移动方法
            offboard_tools.hover_drone,
            ros2_tools.stop_drone_movement,
            uav_tools.get_drone_status,
            uav_tools.emergency_stop,
        ]
        
        # 创建提示词模板
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """You are an intelligent UAV (Unmanned Aerial Vehicle) agent controlling a PX4-based drone.

SAFETY IS PARAMOUNT. You must follow these critical safety protocols:
1. ALWAYS check drone status using get_drone_status() before executing any command.
2. NEVER takeoff without first arming the drone using arm_drone().
3. NEVER move the drone unless it is confirmed to be in the air.
4. ALWAYS verify the drone is connected using connect_drone() before any operation.
5. ALWAYS USE THE AVAILABLE TOOLS: connect_drone(), arm_drone(), takeoff_drone(), land_drone(), move_drone(), get_drone_status(), disarm_drone(), return_to_launch(), emergency_stop().

Flight operations must follow this sequence using the available tools:
1. connect_drone() → 2. arm_drone() → 3. takeoff_drone() OR takeoff_drone_to_altitude() → 4. move_drone() operations → 5. land_drone() → 6. disarm_drone().
NEVER skip steps in this sequence. ALWAYS use the specific tool functions.
For 'Arm the drone and take off' commands, you MUST call both arm_drone() AND takeoff_drone() functions.
For takeoff with specific altitude (like "起飞至1m高度" or "takeoff to 1 meter"), use takeoff_drone_to_altitude(altitude=X).
For movement commands (like "向前飞10m" or "move forward 10 meters"), ALWAYS follow this sequence:
1. First call get_drone_status() to check real-time status
2. If drone is in air, use move_drone_offboard(direction, distance) - this is the PREFERRED movement method for reliable control
3. Alternative: use move_drone_ros2(direction, distance) if offboard fails
4. NEVER use move_drone_distance() as it has import errors
5. NEVER use move_drone() for distance-based movement
NEVER provide generic instructions - ALWAYS execute the actual tool functions."""),
            ("human", "{input}"),
            ("placeholder", "{agent_scratchpad}"),
        ])
        
        # 创建代理
        self.agent = create_tool_calling_agent(self.__llm, self.tools, self.prompt)
        self.agent_executor = AgentExecutor(agent=self.agent, tools=self.tools, verbose=True)
        
        self.examples = [
            "Connect to the drone and check status",
            "Arm the drone and take off",
            "解锁无人机随后起飞至1米高度",
            "Takeoff to 5 meters altitude",
            "Fly forward for 3 seconds then land",
            "往左飞10米",
            "Move up 2 seconds, then move down 2 seconds",
            "Land the drone safely",
            "Check current drone status",
            "Emergency stop the drone",
        ]
        
        self.last_events = []

    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the Simple ROSA-UAV agent 🚁🤖. I can help you control PX4 drones safely using natural language!\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            "Try 'help', 'examples' or 'exit'. ",
            style="italic",
        )
        greeting.append(
            "Remember: Safety first! Always follow proper flight procedures.",
            style="bold red",
        )
        return greeting

    def choose_example(self):
        """Get user selection from the list of examples."""
        return pyip.inputMenu(
            self.examples,
            prompt="\nChoose a flight example and press enter: \n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1",
        )

    def get_input(self, prompt: str):
        """Get user input from the console."""
        return pyip.inputStr(prompt, default="help")

    def invoke(self, query: str):
        """Invoke the agent with a query."""
        try:
            result = self.agent_executor.invoke({"input": query})
            return result.get("output", "No response")
        except Exception as e:
            return f"Error: {e}"

    async def run(self):
        """Run the UAV agent's main interaction loop."""
        console = Console()
        
        # Check UAV system connection
        console.print("[yellow]Checking UAV system connection...[/yellow]")
        try:
            connection_status = uav_tools.connect_drone.invoke("")
            console.print(f"[green]{connection_status}[/green]")
        except Exception as e:
            console.print(f"[red]Failed to check UAV connection: {e}[/red]")
            console.print("[yellow]You can still use the agent, but drone commands may fail.[/yellow]")

        while True:
            console.print(self.greeting)
            input_text = self.get_input("> ")

            # Handle special commands
            if input_text == "exit":
                console.print("[yellow]Shutting down UAV agent. Fly safe! ✈️[/yellow]")
                break
            elif input_text == "help":
                help_text = """
# UAV Agent Help

## Available Commands:
- **help**: Show this help message
- **examples**: Show example commands
- **exit**: Exit the agent

## Example Commands:
- "Connect to the drone and check status"
- "Arm the drone and take off"
- "Fly forward for 3 seconds then land"
- "Land the drone safely"
- "Check current drone status"

## Safety Notes:
- Always arm before takeoff
- Always land before disarming
- Check status regularly
                """
                console.print(Panel(Markdown(help_text), title="Help", border_style="blue"))
            elif input_text == "examples":
                example = self.choose_example()
                console.print(f"[green]Executing example: {example}[/green]")
                response = self.invoke(example)
                console.print(Panel(Markdown(response), title="UAV Response", border_style="green"))
            else:
                response = self.invoke(input_text)
                console.print(Panel(Markdown(response), title="UAV Response", border_style="green"))

def main():
    """Main function to run the simple UAV agent."""
    dotenv.load_dotenv(dotenv.find_dotenv())
    
    agent = SimpleUAVAgent(streaming=True)
    asyncio.run(agent.run())

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[yellow]UAV Agent interrupted by user. Goodbye! ✈️[/yellow]")
    except Exception as e:
        print(f"\n[red]Error: {e}[/red]")
        import traceback
        traceback.print_exc()
