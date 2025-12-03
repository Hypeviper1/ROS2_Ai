#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rosa import ROSA, RobotSystemPrompts

# Tools (ROS2 versions)
from tool import (
    spawn_turtle,
    kill_turtle,
    clear_turtlesim,
    get_turtle_pose,
    teleport_absolute,
    teleport_relative,
    publish_twist_to_cmd_vel,
    stop_turtle,
    reset_turtlesim,
    set_pen,
    has_moved_to_expected_coordinates,
)

# LLM (Azure or OpenAI)
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from azure_setup import llm


class RosaTurtleAgent(Node):

    def __init__(self):
        super().__init__('rosa_turtle_agent')

        self.tools = [
            spawn_turtle,
            kill_turtle,
            clear_turtlesim,
            get_turtle_pose,
            teleport_absolute,
            teleport_relative,
            publish_twist_to_cmd_vel,
            stop_turtle,
            reset_turtlesim,
            set_pen,
            has_moved_to_expected_coordinates,
        ]

        self.rosa = ROSA(
            ros_version=2,
            llm=llm,
            tools=self.tools,
            prompts=RobotSystemPrompts(),
            verbose=True,
        )

        print("ROSA TurtleSim Ready! (ROS2 Jazzy)")
        print("Type natural commands. Press CTRL + C to exit.")

    def run_console(self):
        while rclpy.ok():
            try:
                user_inp = input("\nUser: ")
                # ROSA exposes an `invoke` method for non-streaming calls.
                response = self.rosa.invoke(user_inp)
                print("\n--- ROSA Response ---")
                print(response)
            except KeyboardInterrupt:
                print("\nExiting...")
                break


def main():
    rclpy.init()

    agent = RosaTurtleAgent()
    agent.run_console()

    agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
