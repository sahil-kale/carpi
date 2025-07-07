#!/usr/bin/env python3

import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

class JoyPublisherNode(Node):
    def __init__(self):
        super().__init__('teleop_joy_node')
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected.")
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info("Teleop Joy Node has been started.")

    def tick(self):
        pygame.event.pump()
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        msg.buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        self.pub.publish(msg)

    def shutdown(self):
        pygame.quit()

def main(args=None):
    rclpy.init()
    node = JoyPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
