#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_controller_interface.msg import RobotCommandRaw
from robot_controller_sdk import Board
import numpy as np

class RobotControllerInterface(Node):
    def __init__(self):
        super().__init__('robot_controller_interface')
        self.board = Board()
        self.board.enable_reception()
        self.board.set_buzzer(1900, 0.05, 0.01, 1) # aural feedback

        self.get_logger().info("Robot Controller Interface Node has been started.")

        # HW specific parameters
        self.servo_id = 1
        self.motor_l_id = 2
        self.motor_r_id = 4

        self.invert_motor_l = False
        self.invert_motor_r = True

        # subscribe to command topic
        self.command_subscriber = self.create_subscription(
            RobotCommandRaw,
            "~/robot_command",
            self.command_callback,
            1
        )
    
    def command_callback(self, msg: RobotCommandRaw):
        self.get_logger().debug(f"Received command: {msg.command}")

        SERVO_PWM_MIN_US = 1000
        SERVO_PWM_MAX_US = 2000

        # interpolate command from [-1, 1] to [SERVO_PWM_MIN_US, SERVO_PWM_MAX_US]
        pwm_value = int(np.interp(msg.cmd_steer, [-1, 1], [SERVO_PWM_MIN_US, SERVO_PWM_MAX_US]))
        self.board.pwm_servo_set_position(msg.cmd_steer_target_ramp_time_s, [[self.servo_id, pwm_value]])
        
        # use the same command wheel rps for both motors!
        motor_l_rps = msg.cmd_wheel_rps * (-1 if self.invert_motor_l else 1)
        motor_r_rps = msg.cmd_wheel_rps * (-1 if self.invert_motor_r else 1)

        motor_commands = [
            [self.motor_l_id, motor_l_rps],
            [self.motor_r_id, motor_r_rps]
        ]

        self.board.set_motor_speed(motor_commands)

    def shutdown(self):
        self.get_logger().info("Shutting down Robot Controller Interface Node.")
        # stop all motors
        self.board.set_motor_speed([[self.motor_l_id, 0], [self.motor_r_id, 0]])

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotControllerInterface()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()