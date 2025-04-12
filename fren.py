#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')
        
        # Parametreleri al
        self.declare_parameter('stop_distance', 1.0)  # 1 metre varsayılan
        self.declare_parameter('laser_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_speed', 0.2)
        
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        laser_topic = self.get_parameter('laser_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        
        # Abonelik ve yayıncı oluştur
        self.subscription = self.create_subscription(
            LaserScan,
            laser_topic,
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        self.get_logger().info(f'Obstacle stopper initialized with stop distance: {self.stop_distance}m')
    
    def scan_callback(self, msg):
        # NaN ve inf değerlerini göz ardı ederek en yakın engeli bul
        filtered_ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r)]
        
        if not filtered_ranges:
            self.get_logger().warn("No valid range data received")
            return
        
        min_distance = min(filtered_ranges)
        
        cmd_vel = Twist()
        
        if min_distance <= self.stop_distance:
            # Engel çok yakın, dur
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m - STOPPING!')
        else:
            # Engel yok veya yeterince uzak, normal hareket
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = 0.0
        
        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    obstacle_stopper = ObstacleStopper()
    rclpy.spin(obstacle_stopper)
    obstacle_stopper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
