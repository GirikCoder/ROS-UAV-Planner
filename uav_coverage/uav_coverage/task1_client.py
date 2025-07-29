#!/usr/bin/env python3
import math
from itertools import combinations
import rclpy
from rclpy.node import Node
from my_uav_interfaces.srv import GenerateUAVs
from my_uav_interfaces.msg import UAVInfo, UAVCollection


class Task1Client(Node):
    def __init__(self):
        super().__init__('task1_client')
        
        # Create service client
        self.client = self.create_client(GenerateUAVs, 'generate_uavs')
        
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for UAV generation service...')
        
        self.get_logger().info("Task 1 Client ready!")

    def request_uavs(self, count):
        """Request UAVs from server"""
        request = GenerateUAVs.Request()
        request.count = count
        
        self.get_logger().info(f"Requesting {count} UAVs...")
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            response = future.result()
            self.display_uavs(response.uav_list)
            self.check_intersections(response.uav_list)
        else:
            self.get_logger().error("Service call failed!")

    def display_uavs(self, uav_collection):
        """Display received UAVs"""
        print(f"\n Received {uav_collection.count} UAVs:")
        print("=" * 50)
        
        for uav in uav_collection.uavs:
            print(f"UAV {uav.id}: Position ({uav.x:.1f}, {uav.y:.1f}) | Radius: {uav.radius:.1f}")

    def check_intersections(self, uav_collection):
        """Check for coverage intersections"""
        print(f"\n Checking for intersections:")
        print("-" * 30)
        
        intersections_found = False
        
        # Check all pairs of UAVs
        for uav1, uav2 in combinations(uav_collection.uavs, 2):
            distance = math.sqrt((uav2.x - uav1.x)**2 + (uav2.y - uav1.y)**2)
            
            if distance < (uav1.radius + uav2.radius):
                print(f"UAV {uav1.id} intersects with UAV {uav2.id} (distance: {distance:.1f})")
                intersections_found = True
        
        if not intersections_found:
            print("No intersections found - Good coverage!")
        
        # Calculate coverage stats
        self.calculate_coverage_stats(uav_collection)

    def calculate_coverage_stats(self, uav_collection):
        """Calculate and display coverage statistics"""
        if len(uav_collection.uavs) < 2:
            return
        
        distances = []
        for uav1, uav2 in combinations(uav_collection.uavs, 2):
            distance = math.sqrt((uav2.x - uav1.x)**2 + (uav2.y - uav1.y)**2)
            distances.append(distance)
        
        avg_distance = sum(distances) / len(distances)
        min_distance = min(distances)
        max_distance = max(distances)
        
        print(f"\n Coverage Statistics:")
        print(f"   Average distance: {avg_distance:.1f}")
        print(f"   Minimum distance: {min_distance:.1f}")
        print(f"   Maximum distance: {max_distance:.1f}")


def main(args=None):
    rclpy.init(args=args)
    node = Task1Client()
    
    # Request 10 UAVs (you can change this number)
    node.request_uavs(count=8)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()