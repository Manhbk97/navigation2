#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from collections import deque
import time

class OdomDelay(Node):
    def __init__(self):
        super().__init__('odom_delay')
        self.delay = 0.2  # seconds
        self.buffer = deque()

        self.sub = self.create_subscription(
            Odometry, 'odometry_raw', self.cb, 10)
        self.pub = self.create_publisher(
            Odometry, 'ekf_odom', 10)
        self.timer = self.create_timer(0.02, self.publish_delayed)

    def cb(self, msg):
        self.buffer.append((time.time(), msg))

    def publish_delayed(self):
        now = time.time()
        while self.buffer and (now - self.buffer[0][0]) >= self.delay:
            _, msg = self.buffer.popleft()
            self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(OdomDelay())