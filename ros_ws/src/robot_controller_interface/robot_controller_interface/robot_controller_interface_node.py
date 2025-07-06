#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_controller_interface_msgs.msg import RobotCommandRaw, RobotSensorTelemetry
from robot_controller_interface.robot_controller_sdk import Board
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

        self.invert_motor_l = True
        self.invert_motor_r = False

        self.command_subscriber = self.create_subscription(
            RobotCommandRaw,
            "~/robot_command",
            self.command_callback,
            1
        )

        self.robot_sensor_telemetry_publisher = self.create_publisher(
            RobotSensorTelemetry,
            "~/robot_sensor_telemetry",
            1
        )

        SENSOR_TELEMETRY_PERIOD_S = 20/1000

        self.robot_sensor_telemetry_publisher_timer = self.create_timer(
            SENSOR_TELEMETRY_PERIOD_S,
            self.robot_sensor_telemetry_publisher_timer_callback
        )
    
    def command_callback(self, msg: RobotCommandRaw):
        self.get_logger().debug(f"Received command: {msg}")

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

    def robot_sensor_telemetry_publisher_timer_callback(self):
        imu_data = self.board.get_imu()
        motor_rps_actual_all_4_motors = self.board.get_motor_rps_actual()

        GRAVITY_M_PER_S_SQUARED = 9.80665 # IMU gives acceleration in g's

        if imu_data is None or motor_rps_actual_all_4_motors is None:
            failed_to_get_imu_data = imu_data is None
            failed_to_get_motor_rps_data = motor_rps_actual_all_4_motors is None
            self.get_logger().warn(f"Failed to get {'IMU data' if failed_to_get_imu_data else ''}{' and ' if failed_to_get_imu_data and failed_to_get_motor_rps_data else ''}{'motor RPS data' if failed_to_get_motor_rps_data else ''} from robot controller board.")
            return
        msg = RobotSensorTelemetry()
        # IMU code provided by HiWonder and adapted here
        ax, ay, az, gx, gy, gz = imu_data
        msg.imu.linear_acceleration.x = ax * GRAVITY_M_PER_S_SQUARED
        msg.imu.linear_acceleration.y = ay * GRAVITY_M_PER_S_SQUARED
        msg.imu.linear_acceleration.z = az * GRAVITY_M_PER_S_SQUARED
        msg.imu.angular_velocity.x = gx
        msg.imu.angular_velocity.y = gy
        msg.imu.angular_velocity.z = gz

        msg.imu.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
        msg.imu.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                            0.0, 0.01, 0.0,
                                            0.0, 0.0, 0.01]
        msg.imu.linear_acceleration_covariance = [0.0004, 0.0, 0.0,
                                                0.0, 0.0004, 0.0,
                                                0.0, 0.0, 0.004]
        
        # Consider adding a single channel to report the average of L/R motor
        motor_rps_l = motor_rps_actual_all_4_motors[self.motor_l_id - 1] * (-1 if self.invert_motor_l else 1)
        motor_rps_r = motor_rps_actual_all_4_motors[self.motor_r_id - 1] * (-1 if self.invert_motor_r else 1)                                
        msg.motor_l_rps_actual = motor_rps_l
        msg.motor_r_rps_actual = motor_rps_r

        self.robot_sensor_telemetry_publisher.publish(msg)
        self.get_logger().debug(f"Published sensor telemetry: {msg}")


    def shutdown(self):
        self.get_logger().info("Shutting down Robot Controller Interface Node.")
        # stop all motors
        self.board.set_motor_speed([[self.motor_l_id, 0], [self.motor_r_id, 0]])

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotControllerInterface()
    try: 
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()