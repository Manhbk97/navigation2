#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import math


class VelocityCalculatorNode(Node):
    def __init__(self):
        super().__init__('velocity_calculator_node')
        
        # Robot parameters
        self.wheel_base = 0.373  # Distance between wheels (meters)
        self.wheel_radius = 0.0855  # Wheel radius (meters)
        
        # Encoder parameters
        self.encoder_resolution = 65536  # Pulses per revolution (adjust based on your encoder)
        
        # Previous encoder values
        self.prev_left_encoder = None
        self.prev_right_encoder = None
        self.prev_time = None

        # Logging throttle
        self.last_log_time = None
        self.log_interval = 1.0  # Log every 1 second (change to 2.0 for 2 seconds)
        
        # Create QoS profile with BEST_EFFORT reliability
        # This matches typical sensor data publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Changed from RELIABLE to BEST_EFFORT
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriber to motor info with custom QoS
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/motor/info',
            self.motor_info_callback,
            qos_profile  # Using custom QoS profile
        )
        
        # Publisher for velocity with standard QoS
        publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/calculated_velocity',
            publisher_qos
        )
        
        self.get_logger().info('Velocity Calculator Node Started')
        self.get_logger().info(f'Wheel base: {self.wheel_base} m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info('QoS: Using BEST_EFFORT reliability for /motor/info subscription')
    
    def motor_info_callback(self, msg):
        """
        Callback function for /motor/info topic
        Extracts encoder values and calculates velocities
        """
        # Based on your example, encoders are at indices 5 and 6
        # data[5] = left encoder = -162300
        # data[6] = right encoder = 113536
        
        if len(msg.data) < 7:
            self.get_logger().warn('Motor info message does not have enough data')
            return
        
        left_encoder = msg.data[5]
        right_encoder = msg.data[6]
        
        current_time = self.get_clock().now()
        
        # Calculate velocities only after we have previous values
        if self.prev_left_encoder is not None and self.prev_right_encoder is not None:
            # Calculate time difference
            dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
            
            if dt > 0:
                # Calculate encoder differences
                delta_left = left_encoder - self.prev_left_encoder
                delta_right = right_encoder - self.prev_right_encoder
                
                # Convert encoder ticks to wheel rotations
                left_rotations = delta_left / self.encoder_resolution
                right_rotations = delta_right / self.encoder_resolution
                
                # Calculate distance traveled by each wheel
                left_distance = left_rotations * 2 * math.pi * self.wheel_radius
                right_distance = right_rotations * 2 * math.pi * self.wheel_radius
                
                # Calculate wheel velocities
                left_velocity = -left_distance / dt
                right_velocity = right_distance / dt
                
                # Calculate robot linear and angular velocities
                # For differential drive:
                # linear_velocity = (v_right + v_left) / 2
                # angular_velocity = (v_right - v_left) / wheel_base
                
                linear_velocity = (right_velocity + left_velocity) / 2.0
                angular_velocity = (right_velocity - left_velocity) / self.wheel_base

                # Log the results (throttled to log_interval)
                should_log = False
                if self.last_log_time is None:
                    should_log = True
                else:
                    time_since_last_log = (current_time - self.last_log_time).nanoseconds / 1e9
                    if time_since_last_log >= self.log_interval:
                        should_log = True

                if should_log:
                    self.get_logger().info('='*50)
                    self.get_logger().info(f'Left Encoder: {left_encoder}, Right Encoder: {right_encoder}')
                    self.get_logger().info(f'Delta Time: {dt:.4f} s')
                    self.get_logger().info(f'Left Velocity: {left_velocity:.4f} m/s')
                    self.get_logger().info(f'Right Velocity: {right_velocity:.4f} m/s')
                    self.get_logger().info(f'Linear Velocity: {linear_velocity:.4f} m/s')
                    self.get_logger().info(f'Angular Velocity: {angular_velocity:.4f} rad/s')
                    self.last_log_time = current_time
                
                # Publish velocity as Twist message
                twist_msg = Twist()
                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity
                self.velocity_publisher.publish(twist_msg)
        
        # Update previous values
        self.prev_left_encoder = left_encoder
        self.prev_right_encoder = right_encoder
        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCalculatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
