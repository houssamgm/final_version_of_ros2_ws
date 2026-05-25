#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class DynamicObstacles(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles')
        
        self.num_obstacles = 10 # Updated to 10
        self.publishers_ = []
        
        for i in range(1, self.num_obstacles + 1):
            topic_name = f'/obs{i}/cmd_vel'
            pub = self.create_publisher(Twist, topic_name, 10)
            self.publishers_.append(pub)
            self.get_logger().info(f'Created velocity publisher for: {topic_name}')

        self.timer = self.create_timer(0.1, self.move_obstacles)
        self.start_time = time.time()

    def move_obstacles(self):
        elapsed_time = time.time() - self.start_time
        
        for i, pub in enumerate(self.publishers_):
            msg = Twist()
            
            base_forward_speed = 0.15  
            base_turn_speed = 0.25 
            
            # Wobble math remains the same, it scales automatically
            wobble = 0.15 * math.sin(elapsed_time * 0.5 + i)
            
            msg.linear.x = base_forward_speed
            msg.angular.z = base_turn_speed + wobble
                
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping dynamic obstacles...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()