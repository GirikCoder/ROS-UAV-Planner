#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from my_uav_interfaces.srv import GenerateUAVs
from my_uav_interfaces.msg import UAVInfo, UAVCollection


class Task1Server(Node):
    def __init__(self):
        super().__init__('task1_server')
        
        # Create service
        self.srv = self.create_service(
            GenerateUAVs, 
            'generate_uavs', 
            self.handle_request
        )

        self.initial_uav_publisher = self.create_publisher(
            UAVCollection, 
            'initial_uav_placement', 
            10
        )
        
        
        # Workspace parameters
        self.screen_width = 500
        self.screen_height = 500
        self.uav_radius = 50
        
        self.get_logger().info("Task 1 UAV Server ready!")

    def handle_request(self, request, response):
        """Generate UAVs at random positions"""
        count = request.count
        self.get_logger().info(f"Generating {count} UAVs")
        
        # Create UAV collection
        uav_collection = UAVCollection()
        uav_collection.count = count
        uav_collection.uavs = []
        
        # Generate UAVs
        for i in range(count):
            uav = UAVInfo()
            uav.id = i
            uav.x = float(random.randint(self.uav_radius, self.screen_width - self.uav_radius))
            uav.y = float(random.randint(self.uav_radius, self.screen_height - self.uav_radius))
            uav.radius = float(self.uav_radius)
            
            uav_collection.uavs.append(uav)
        
        response.uav_list = uav_collection
        self.initial_uav_publisher.publish(uav_collection)
        self.get_logger().info(f"📤 Published {count} initial UAVs for Task 2")
        self.get_logger().info(f"Generated {count} UAVs successfully")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Task1Server()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()