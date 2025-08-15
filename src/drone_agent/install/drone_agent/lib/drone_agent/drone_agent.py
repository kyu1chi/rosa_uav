#!/usr/bin/env python3

import asyncio
import dotenv
import rclpy
from rclpy.node import Node
from langchain_openai import ChatOpenAI
from langchain.agents import tool, Tool
from rich.console import Console
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosa import ROSA

import sys
import os

# Add the scripts directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
print(script_dir)
sys.path.insert(0, script_dir)

import tools.drone as drone_tools
from help import get_help
from llm import get_llm
from prompts import get_prompts


@tool
def emergency_land():
    """Emergency land the drone immediately."""
    return "Emergency landing initiated! The drone will land immediately for safety."


class DroneAgent(ROSA):

    def __init__(self, streaming: bool = False, verbose: bool = True):
        self.__blacklist = ["master", "docker"]
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming)
        self.__streaming = streaming
        
        # Emergency tool
        emergency_tool = Tool(
            name="emergency_land",
            func=self.emergency_land,
            description="Emergency land the drone immediately for safety.",
        )

        super().__init__(
            ros_version=2,
            llm=self.__llm,
            tools=[emergency_land, emergency_tool],
            tool_packages=[drone_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
        )

        self.examples = [
            "Arm the drone and take off to 5 meters",
            "Move forward for 3 seconds",
            "Rotate the drone 90 degrees clockwise",
            "Land the drone safely",
            "Check the drone's current status",
            "Move in a square pattern",
            "Hover at current position for 10 seconds",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear(),
            "emergency": lambda: self.submit("Emergency land the drone immediately"),
        }

    def emergency_land(self, input: str):
        return f"""
        EMERGENCY LANDING ACTIVATED!
        
        <ROSA_INSTRUCTIONS>
            You should immediately use the emergency_land tool to land the drone safely.
            Do not perform any other actions until the drone has landed.
        </ROSA_INSTRUCTIONS>
        """

    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSA-Drone agent 🚁🤖. How can I help you control the drone today?\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            f"Try {', '.join(self.command_handler.keys())} or exit.",
            style="italic",
        )
        greeting.append(
            "\n⚠️  Safety first: Type 'emergency' for immediate landing.",
            style="bold red",
        )
        return greeting

    def choose_example(self):
        console = Console()
        console.print("Choose an example:")
        for i, example in enumerate(self.examples):
            console.print(f"{i + 1}. {example}")
        
        try:
            choice = int(input("Enter choice (1-{}): ".format(len(self.examples))))
            if 1 <= choice <= len(self.examples):
                return self.examples[choice - 1]
            else:
                return "Invalid choice. Please try again."
        except ValueError:
            return "Invalid input. Please enter a number."

    def get_input(self, prompt: str) -> str:
        return input(prompt)

    async def clear(self):
        console = Console()
        console.clear()

    async def run(self):
        """Run the DroneAgent's main interaction loop."""
        await self.clear()
        console = Console()

        # while True:
        #     console.print(self.greeting)
        #     #user_input = self.get_input("> ")
        #     user_input = "让无人机起飞至2m高度"
        #     # Handle special commands
        #     if user_input == "exit":
        #         break
        #     elif user_input in self.command_handler:
        #         await self.command_handler[user_input]()
        #     else:
        #         await self.submit(user_input)
        console.print(self.greeting)
        #user_input = self.get_input("> ")
        user_input = "让无人机起飞至10m高度，在往飞走10m，然后降落"
        # Handle special commands
        await self.submit(user_input)

    async def submit(self, query: str):
        if self.__streaming:
            await self.stream_response(query)
        else:
            self.print_response(query)

    def print_response(self, query: str):
        """Submit the query to the agent and print the response."""
        response = self.invoke(query)
        console = Console()
        
        with Live(console=console, auto_refresh=True, vertical_overflow="visible") as live:
            content_panel = Panel(
                Markdown(response), title="Drone Response", border_style="green"
            )
            live.update(content_panel, refresh=True)

    async def stream_response(self, query: str):
        """Stream the response from the agent."""
        console = Console()
        content = ""
        
        with Live(console=console, auto_refresh=True, vertical_overflow="visible") as live:
            async for chunk in self.astream({"input": query}):
                if "output" in chunk:
                    content += chunk["output"]
                    content_panel = Panel(
                        Markdown(content), title="Drone Response (Streaming)", border_style="blue"
                    )
                    live.update(content_panel, refresh=True)


def main():
    dotenv.load_dotenv(dotenv.find_dotenv())
    rclpy.init()
    
    node = rclpy.create_node('rosa_drone_agent')
    streaming = node.get_parameter_or('streaming', rclpy.Parameter('streaming', value=False)).value
    connection_string = node.get_parameter_or('connection_string', rclpy.Parameter('connection_string', value='udp://:14540')).value  # 修改端口匹配PX4
    
    # 初始化无人机节点
    drone_tools.init_drone_node()
    
    drone_agent = DroneAgent(verbose=False, streaming=streaming)
    
    # 自动连接无人机
    print(f"自动连接无人机: {connection_string}")
    drone_tools.connect_drone(connection_string)
    
    try:
        asyncio.run(drone_agent.run())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
