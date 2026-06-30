#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Twist
import math


class VelocityCalculatorNode(Node):
    def __init__(self):
        super().__init__('feedback_odom_encoder_node')
        
        # Robot parameters
        self.wheel_base = 0.373  # Distance between wheels (meters)
        self.wheel_radius = 0.0855  # Wheel radius (meters)
        self.max_rpm = 250.0  # Maximum RPM limit
        
        # Encoder parameters
        self.encoder_resolution = 65536  # Pulses per revolution (adjust based on your encoder)
        
        # Previous encoder values
        self.prev_left_encoder = None
        self.prev_right_encoder = None
        self.prev_time = None

        # Logging throttle
        self.last_log_time = None
        self.log_interval = 1.0  # Log every 1 second (change to 2.0 for 2 seconds)

        # Store calculated RPM for comparison
        self.calculated_left_rpm = 0.0
        self.calculated_right_rpm = 0.0
        
        # QoS profile for motor/info (BEST_EFFORT to match sensor data)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # QoS profile for filtered_key_vel (RELIABLE to match collision_monitor)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriber to motor info with BEST_EFFORT QoS
        self.motor_info_subscription = self.create_subscription(
            Int32MultiArray,
            'motor/info',
            self.motor_info_callback,
            qos_best_effort
        )

        # Subscriber to filtered_key_vel with RELIABLE QoS
        self.filtered_vel_subscription = self.create_subscription(
            Twist,
            'filtered_key_vel',
            self.rpm_filtered_key_vel_callback,
            qos_reliable
        )
        
        # Publisher for velocity with standard QoS
        publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.velocity_publisher = self.create_publisher(
            Twist,
            'calculated_velocity',
            publisher_qos
        )

        # Publisher for motor RPM
        self.rpm_publisher = self.create_publisher(
            Int32MultiArray,
            'motor/rpm',
            publisher_qos
        )

        # Publishers for individual wheel RPM (for rqt_plot visualization)
        self.calculated_left_rpm_pub = self.create_publisher(
            Float32,
            'rpm/calculated/left',
            publisher_qos
        )
        self.calculated_right_rpm_pub = self.create_publisher(
            Float32,
            'rpm/calculated/right',
            publisher_qos
        )
        self.actual_left_rpm_pub = self.create_publisher(
            Float32,
            'rpm/actual/left',
            publisher_qos
        )
        self.actual_right_rpm_pub = self.create_publisher(
            Float32,
            'rpm/actual/right',
            publisher_qos
        )

        self.get_logger().info('Velocity Calculator Node Started')
        self.get_logger().info(f'Wheel base: {self.wheel_base} m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info('QoS: motor/info=BEST_EFFORT, filtered_key_vel=RELIABLE')
    
    def motor_info_callback(self, msg):
        """
        Callback function for /motor/info topic
        Extracts encoder values and calculates velocities
        """
        # Based on your example, encoders are at indices 5 and 6
        # data[5] = left encoder = -162300
        # data[6] = right encoder = 113536
        # data[7] = left RPM (actual from motor)
        # data[8] = right RPM (actual from motor)

        if len(msg.data) < 9:
            self.get_logger().warn('Motor info message does not have enough data')
            return

        left_encoder = msg.data[5]
        right_encoder = msg.data[6]
        actual_left_rpm = msg.data[7]
        actual_right_rpm = msg.data[8]

        # Publish actual RPM as scalar topics (divided by 10 to get actual RPM)
        actual_left_rpm_msg = Float32()
        actual_left_rpm_msg.data = float(actual_left_rpm)
        self.actual_left_rpm_pub.publish(actual_left_rpm_msg)

        actual_right_rpm_msg = Float32()
        actual_right_rpm_msg.data = float(actual_right_rpm)
        self.actual_right_rpm_pub.publish(actual_right_rpm_msg)

        current_time = self.get_clock().now()
        
        # Calculate velocity directly from RPM (simpler, no time delta needed)
        # left_rpm = actual_left_rpm / 10.0
        # right_rpm = actual_right_rpm / 10.0

        # left_velocity = -(left_rpm / 60.0) * 2 * math.pi * self.wheel_radius
        # right_velocity = (right_rpm / 60.0) * 2 * math.pi * self.wheel_radius

        # linear_velocity = (right_velocity + left_velocity) / 2.0
        # angular_velocity = (right_velocity - left_velocity) / self.wheel_base
        # # Publish velocity as Twist message
        # twist_msg = Twist()
        # twist_msg.linear.x = linear_velocity
        # twist_msg.angular.z = angular_velocity
        # self.velocity_publisher.publish(twist_msg)

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
                    self.get_logger().info('--- RPM Comparison ---')
                    self.get_logger().info(f'Actual Motor RPM:     L={actual_left_rpm:6d}, R={actual_right_rpm:6d}')
                    self.get_logger().info(f'Calculated RPM (cmd): L={self.calculated_left_rpm:6.1f}, R={self.calculated_right_rpm:6.1f}')
                    rpm_diff_left = abs(actual_left_rpm/10 - self.calculated_left_rpm)
                    rpm_diff_right = abs(actual_right_rpm/10 - self.calculated_right_rpm)
                    self.get_logger().info(f'RPM Difference:       L={rpm_diff_left:6.1f}, R={rpm_diff_right:6.1f}')
                    self.last_log_time = current_time
                
                # Publish velocity as Twist message
                twist_msg = Twist()
                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity
                self.velocity_publisher.publish(twist_msg)
        
        # # Update previous values
        # self.prev_left_encoder = left_encoder
        # self.prev_right_encoder = right_encoder
        # self.prev_time = current_time

    def rpm_filtered_key_vel_callback(self, msg):
        """
        Callback function for filtered_key_vel topic (Twist message)
        Converts linear and angular velocities to motor RPM
        """
        # Extract linear and angular velocities
        linear_vel = msg.linear.x   # m/s
        angular_vel = msg.angular.z  # rad/s

        # Convert to wheel speeds using differential drive kinematics
        left_wheel_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_wheel_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert wheel velocities to RPM
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        left_rpm = (left_wheel_vel / wheel_circumference) * 60.0
        right_rpm = (right_wheel_vel / wheel_circumference) * 60.0

        # Clamp to maximum RPM
        left_rpm = max(min(left_rpm, self.max_rpm), -self.max_rpm)
        right_rpm = max(min(right_rpm, self.max_rpm), -self.max_rpm)

        # Store calculated RPM for comparison in motor_info_callback
        self.calculated_left_rpm = left_rpm
        self.calculated_right_rpm = right_rpm

        # Publish motor RPM as Int32MultiArray
        rpm_msg = Int32MultiArray()
        rpm_msg.data = [int(left_rpm*10), int(right_rpm*10)]
        self.rpm_publisher.publish(rpm_msg)

        # Publish calculated RPM as scalar topics (for rqt_plot)
        calculated_left_rpm_msg = Float32()
        calculated_left_rpm_msg.data = float(-10 * left_rpm)
        self.calculated_left_rpm_pub.publish(calculated_left_rpm_msg)

        calculated_right_rpm_msg = Float32()
        calculated_right_rpm_msg.data = float(10 * right_rpm)
        self.calculated_right_rpm_pub.publish(calculated_right_rpm_msg)

        # Log the calculated RPM values
        self.get_logger().debug(f'Sent motor speeds: L={left_rpm:.1f}, R={right_rpm:.1f} RPM')

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
