#!/usr/bin/env python3
"""
ROSA Robot Agent for URDF Robot Control
Supports any URDF robot in ROS2 Jazzy (Gazebo, real hardware, RViz)
"""

import rclpy
from rclpy.node import Node

from rosa import ROSA, RobotSystemPrompts

# Robot control tools
from robot_tools import (
    move_robot,
    stop_robot,
    rotate_robot,
    set_joint_positions,
    get_joint_states,
    get_robot_pose,
    check_position_reached,
)

# LLM configuration
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from azure_setup import llm


class RosaRobotAgent(Node):
    """ROSA Agent for controlling URDF robots via natural language."""

    def __init__(self, robot_name: str = "robot", robot_description: str = ""):
        super().__init__('rosa_robot_agent')

        # Define available tools for robot control
        self.tools = [
            move_robot,
            stop_robot,
            rotate_robot,
            set_joint_positions,
            get_joint_states,
            get_robot_pose,
            check_position_reached,
        ]

        # Create custom prompts for your robot
        custom_prompts = RobotSystemPrompts()
        
        # Add robot-specific context if provided
        if robot_description:
            self.get_logger().info(f"Robot Description: {robot_description}")

        # Initialize ROSA with ROS2 version and tools
        self.rosa = ROSA(
            ros_version=2,
            llm=llm,
            tools=self.tools,
            prompts=custom_prompts,
            verbose=True,
            accumulate_chat_history=True,
        )

        print(f"\n{'='*60}")
        print(f"🤖 ROSA {robot_name.upper()} Agent Ready! (ROS2 Jazzy)")
        print(f"{'='*60}")
        print("\n📋 Available Commands:")
        print("  - Type natural language commands to control the robot")
        print("  - Examples:")
        print("    • 'Move forward 1 meter'")
        print("    • 'Rotate 90 degrees left'")
        print("    • 'What is the current robot pose?'")
        print("    • 'Get all joint states'")
        print("    • 'Stop the robot'")
        print("\n  - Type 'exit' or press CTRL+C to quit")
        print(f"{'='*60}\n")

    def run_console(self):
        """Run the interactive console loop."""
        while rclpy.ok():
            try:
                user_inp = input("\n🗣️  User: ")
                
                # Handle exit command
                if user_inp.lower() in ['exit', 'quit', 'q']:
                    print("\n👋 Goodbye!")
                    break
                
                # Skip empty inputs
                if not user_inp.strip():
                    continue
                
                # Process the command through ROSA
                print("\n🤔 Processing...")
                response = self.rosa.invoke(user_inp)
                
                print("\n" + "─"*60)
                print("🤖 ROSA Response:")
                print("─"*60)
                print(response)
                print("─"*60)
                
            except KeyboardInterrupt:
                print("\n\n👋 Interrupted. Exiting...")
                break
            except Exception as e:
                print(f"\n❌ Error: {e}")
                self.get_logger().error(f"Error processing command: {e}")


def main():
    """Main entry point."""
    # Initialize ROS2
    rclpy.init()

    # Configuration - modify these for your robot
    ROBOT_NAME = "my_robot"
    ROBOT_DESCRIPTION = """
    A URDF robot controlled via ROS2.
    - Uses /cmd_vel for velocity control
    - Publishes joint states on /joint_states
    - Publishes odometry on /odom
    """

    try:
        # Create and run the agent
        agent = RosaRobotAgent(
            robot_name=ROBOT_NAME,
            robot_description=ROBOT_DESCRIPTION
        )
        agent.run_console()
    except Exception as e:
        print(f"\n❌ Failed to start agent: {e}")
    finally:
        # Cleanup
        try:
            agent.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
