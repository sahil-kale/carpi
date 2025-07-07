#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_controller_interface_msgs.msg import RobotCommandRaw

class RobotCommandArbitratorNode(Node):
    def __init__(self):
        super().__init__('robot_command_arbitrator_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.robot_command_publisher = self.create_publisher(RobotCommandRaw, '/robot_controller_interface/robot_command', 1)
        self.get_logger().info("Robot Command Arbitrator Node has been started.")

    def joy_callback(self, msg: Joy):
        self.get_logger().debug(f"Received Joy message: {msg.axes}, {msg.buttons}")
        command_msg = RobotCommandRaw()
        axes_horz_0 = msg.axes[0]  # Left stick horizontal
        axes_vert_0 = msg.axes[1]  # Left stick vertical

        command_msg.cmd_steer = axes_horz_0
        command_msg.cmd_wheel_rps = axes_vert_0 * -7.0
        command_msg.cmd_steer_target_ramp_time_s = 0.02
        self.robot_command_publisher.publish(command_msg)
        self.get_logger().debug(f"Published RobotCommandRaw: steer={command_msg.cmd_steer}, wheel_rps={command_msg.cmd_wheel_rps}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandArbitratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
