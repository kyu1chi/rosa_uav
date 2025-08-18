#!/usr/bin/env python3
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import asyncio
import os
from datetime import datetime

import dotenv
import pyinputplus as pyip
try:
    import rclpy
except ImportError:
    print("Warning: rclpy not available. Running in standalone mode.")
    rclpy = None
from langchain.agents import tool, Tool
from rich.console import Console
from rich.console import Group
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosa import ROSA

import tools.uav as uav_tools
from help import get_help
from llm import get_llm
from prompts import get_prompts


# Additional UAV-specific tools
@tool
def flight_pattern_square(side_length: float = 2.0):
    """
    Execute a square flight pattern.
    
    :param side_length: Length of each side in seconds (default: 2.0)
    """
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

@tool
def flight_pattern_circle(radius_time: float = 3.0):
    """
    Execute a circular flight pattern using yaw rotation.
    
    :param radius_time: Time for complete circle in seconds (default: 3.0)
    """
    if radius_time <= 0 or radius_time > 8:
        return "Error: Circle time must be between 0 and 8 seconds for safety."
    
    return f"""
    Executing circular flight pattern over {radius_time} seconds.
    
    <ROSA_INSTRUCTIONS>
        Use manual control to create a circular pattern:
        1. Set manual control with small forward pitch and continuous yaw rotation
        2. Maintain for {radius_time} seconds
        3. Return to hover (all inputs to 0 except throttle 0.5)
    </ROSA_INSTRUCTIONS>
    """


class UAVAgent(ROSA):
    """
    UAV Agent for controlling PX4 drones using natural language through ROSA framework.
    
    This agent provides safe, intuitive control of UAVs while maintaining strict safety protocols.
    """

    def __init__(self, streaming: bool = True, verbose: bool = True):
        self.__blacklist = ["master", "docker"]  # ROS tools to exclude
        self.__prompts = get_prompts()
        self.__llm = get_llm()
        self.__streaming = streaming

        # Additional UAV-specific tools
        hover_tool = Tool(
            name="hover_in_place",
            func=self.hover_in_place,
            description="Make the drone hover in place (neutral position).",
        )

        super().__init__(
            ros_version=2,  # Using ROS2 for PX4 integration
            llm=self.__llm,
            tools=[flight_pattern_square, flight_pattern_circle, hover_tool],
            tool_packages=[uav_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
        )

        self.examples = [
            "Connect to the drone and check status",
            "Arm the drone and take off",
            "Fly forward for 3 seconds then hover",
            "Execute a square flight pattern",
            "Perform a circular flight pattern",
            "Move up 2 seconds, then move down 2 seconds",
            "Land the drone and disarm",
            "Check current drone status",
            "Emergency stop the drone",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear(),
        }

    def hover_in_place(self, input: str = ""):
        """Make the drone hover in place with neutral controls."""
        return """
        Setting drone to hover in place.
        
        <ROSA_INSTRUCTIONS>
            Use set_manual_control with roll=0, pitch=0, throttle=0.5, yaw=0 to maintain hover position.
        </ROSA_INSTRUCTIONS>
        """

    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSA-UAV agent 🚁🤖. I can help you control PX4 drones safely using natural language!\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            f"Try {', '.join(self.command_handler.keys())} or exit. ",
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

    async def clear(self):
        """Clear the chat history."""
        self.clear_chat()
        self.last_events = []
        self.command_handler.pop("info", None)
        os.system("clear")

    def get_input(self, prompt: str):
        """Get user input from the console."""
        console = Console()
        return pyip.inputStr(prompt, default="help")

    async def run(self):
        """
        Run the UAVAgent's main interaction loop.

        This method initializes the console interface and enters a continuous loop to handle user input.
        It processes various commands including 'help', 'examples', 'clear', and 'exit', as well as
        custom user queries for drone control. The method uses asynchronous operations to stream 
        responses and maintain a responsive interface.

        The loop continues until the user inputs 'exit'.

        Returns:
            None

        Raises:
            Any exceptions that might occur during the execution of user commands or streaming responses.
        """
        await self.clear()
        console = Console()

        # Initialize UAV system
        console.print("[yellow]Initializing UAV system...[/yellow]")
        try:
            await uav_tools.initialize_uav_system()
            console.print("[green]UAV system initialized successfully![/green]")
        except Exception as e:
            console.print(f"[red]Failed to initialize UAV system: {e}[/red]")
            console.print("[yellow]You can still use the agent, but drone commands may fail.[/yellow]")

        while True:
            console.print(self.greeting)
            input_text = self.get_input("> ")

            # Handle special commands
            if input_text == "exit":
                console.print("[yellow]Shutting down UAV agent. Fly safe! ✈️[/yellow]")
                break
            elif input_text in self.command_handler:
                await self.command_handler[input_text]()
            else:
                await self.submit(input_text)

    async def submit(self, query: str):
        if self.__streaming:
            await self.stream_response(query)
        else:
            self.print_response(query)

    def print_response(self, query: str):
        """
        Submit the query to the agent and print the response to the console.

        Args:
            query (str): The input query to process.

        Returns:
            None
        """
        response = self.invoke(query)
        console = Console()

        with Live(
            console=console, auto_refresh=True, vertical_overflow="visible"
        ) as live:
            content_panel = Panel(
                Markdown(response), title="UAV Response", border_style="green"
            )
            live.update(content_panel, refresh=True)

    async def stream_response(self, query: str):
        """
        Stream the agent's response with rich formatting.

        This method processes the agent's response in real-time, updating the console
        with formatted output for tokens and keeping track of events.

        Args:
            query (str): The input query to process.

        Returns:
            None

        Raises:
            Any exceptions raised during the streaming process.
        """
        console = Console()
        content = ""
        self.last_events = []

        panel = Panel("", title="UAV Processing", border_style="blue")

        with Live(panel, console=console, auto_refresh=False) as live:
            async for event in self.astream(query):
                event["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[
                    :-3
                ]
                if event["type"] == "token":
                    content += event["content"]
                    panel.renderable = Markdown(content)
                    live.refresh()
                elif event["type"] in ["tool_start", "tool_end", "error"]:
                    self.last_events.append(event)
                elif event["type"] == "final":
                    content = event["content"]
                    if self.last_events:
                        panel.renderable = Markdown(
                            content
                            + "\n\nType 'info' for details on how I executed the commands."
                        )
                    else:
                        panel.renderable = Markdown(content)
                    panel.title = "UAV Response"
                    panel.border_style = "green"
                    live.refresh()

        if self.last_events:
            self.command_handler["info"] = self.show_event_details
        else:
            self.command_handler.pop("info", None)

    async def show_event_details(self):
        """
        Display detailed information about the events that occurred during the last query.
        """
        console = Console()

        if not self.last_events:
            console.print("[yellow]No events to display.[/yellow]")
            return
        else:
            console.print(Markdown("# UAV Command Execution Details"))

        for event in self.last_events:
            timestamp = event["timestamp"]
            if event["type"] == "tool_start":
                console.print(
                    Panel(
                        Group(
                            Text(f"Input: {event.get('input', 'None')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"🔧 Tool Started: {event['name']}",
                        border_style="blue",
                    )
                )
            elif event["type"] == "tool_end":
                console.print(
                    Panel(
                        Group(
                            Text(f"Output: {event.get('output', 'N/A')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"✅ Tool Completed: {event['name']}",
                        border_style="green",
                    )
                )
            elif event["type"] == "error":
                console.print(
                    Panel(
                        Group(
                            Text(f"Error: {event['content']}", style="bold red"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title="❌ Error Occurred",
                        border_style="red",
                    )
                )
            console.print()

        console.print("[bold]End of command execution details[/bold]\n")


def main():
    """Main function to run the UAV agent."""
    dotenv.load_dotenv(dotenv.find_dotenv())

    # Get streaming parameter (default to True for better UX)
    streaming = True
    try:
        if rclpy:
            streaming = rclpy.get_param("~streaming", True)
    except:
        pass  # Use default if ROS parameter not available

    uav_agent = UAVAgent(verbose=False, streaming=streaming)

    asyncio.run(uav_agent.run())


if __name__ == "__main__":
    try:
        if rclpy:
            rclpy.init()
        main()
    except KeyboardInterrupt:
        print("\n[yellow]UAV Agent interrupted by user. Goodbye! ✈️[/yellow]")
    finally:
        try:
            if rclpy:
                rclpy.shutdown()
        except:
            pass
