import rclpy
from rclpy.node import Node

import numpy as np
import math
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float64

MAX_ACCELERATION = 1.5   # m/s²
MAX_DECELERATION = 3.0   # m/s²
A_DES_MIN = -8.0         # reject planner spikes below this (phantom CIPO gave -50)
A_DES_MAX = MAX_ACCELERATION


class CarlaControlPublisher(Node):
    def __init__(self):
        super().__init__('carla_control_publisher')
        self.steering_sub_ = self.create_subscription(Float64, '/vehicle/steering_cmd', self.steering_callback, 1)
        self.throttle_sub_ = self.create_subscription(Float64, '/vehicle/throttle_cmd', self.throttle_callback, 1)
        self.speed_sub_ = self.create_subscription(Float64, '/vehicle/speed', self.speed_callback, 1)
        self.control_pub_ = self.create_publisher(CarlaEgoVehicleControl, "/carla/hero/vehicle_control_cmd", 1)

        self.speed = 0.0
        self.steering_angle_cmd = 0.0
        self.acceleration = 0.0

        # Publish only once BOTH a fresh steering and a fresh throttle have
        # arrived. Each planning cycle in VisionPilot emits one steering + one
        # throttle together, so this fires once per cycle, at the planner's own
        # rate, with no zero-order-hold delay.
        self.have_steering = False
        self.have_throttle = False

        self.last_cmd_time = self.get_clock().now()
        self.watchdog = self.create_timer(0.2, self.watchdog_callback)

    def publish_control(self):
        accel = float(np.clip(self.acceleration, A_DES_MIN, A_DES_MAX))

        msg = CarlaEgoVehicleControl()
        if accel >= 0.0:
            msg.throttle = float(np.clip(accel / MAX_ACCELERATION, 0.0, 1.0))
            msg.brake = 0.0
        else:
            msg.throttle = 0.0
            msg.brake = float(np.clip(-accel / MAX_DECELERATION, 0.0, 1.0))

        # Convert steering angle (tyre angle in radians) to CARLA normalized steering [-1.0, 1.0]
        # For Lincoln MKZ in CARLA, full lock is ~70 deg
        msg.steer = float(np.clip((180.0 / np.pi) * (self.steering_angle_cmd / 70.0), -1.0, 1.0))
        msg.hand_brake = False
        msg.reverse = False
        msg.manual_gear_shift = False
        msg.gear = 1

        self.control_pub_.publish(msg)
        self.last_cmd_time = self.get_clock().now()

    def try_publish(self):
        if self.have_steering and self.have_throttle:
            self.publish_control()
            self.have_steering = False
            self.have_throttle = False

    def steering_callback(self, msg):
        self.steering_angle_cmd = msg.data
        self.have_steering = True
        self.try_publish()

    def throttle_callback(self, msg):
        self.acceleration = msg.data
        self.have_throttle = True
        self.try_publish()

    def speed_callback(self, msg):
        self.speed = msg.data

    def watchdog_callback(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > 0.5:
            msg = CarlaEgoVehicleControl()
            msg.throttle = 0.0
            msg.steer = 0.0
            msg.brake = 1.0
            msg.gear = 1
            self.control_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaControlPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
