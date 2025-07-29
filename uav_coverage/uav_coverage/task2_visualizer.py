#!/usr/bin/env python3
import math
import time
from typing import Dict
import rclpy
from rclpy.node import Node
from my_uav_interfaces.msg import UAVInfo, UAVCollection


class Task2Visualizer(Node):
    def __init__(self):
        super().__init__('task2_visualizer')
        
        # Subscribe to UAV states
        self.uav_states_sub = self.create_subscription(
            UAVCollection, 'current_uav_states', self.uav_states_callback, 10)
        
        # Storage
        self.current_uavs: Dict[int, UAVInfo] = {}
        self.last_update_time = time.time()
        self.update_counter = 0
        
        # Display timer
        self.create_timer(2.0, self.display_status)  # Display every 2 seconds
        
        self.get_logger().info("📺 Task 2 Visualizer started!")

    def uav_states_callback(self, msg: UAVCollection):
        """Update UAV states"""
        self.current_uavs.clear()
        for uav in msg.uavs:
            self.current_uavs[uav.id] = uav
        self.last_update_time = time.time()
        self.update_counter += 1

    def display_status(self):
        """Display current UAV status"""
        if not self.current_uavs:
            return
        
        print("\n" + "="*70)
        print(f"🚁 DYNAMIC UAV STATUS - {len(self.current_uavs)} Active UAVs")
        print(f"📊 Update #{self.update_counter} | Last update: {time.time() - self.last_update_time:.1f}s ago")
        print("="*70)
        
        # Display UAVs sorted by ID
        for uav_id in sorted(self.current_uavs.keys()):
            uav = self.current_uavs[uav_id]
            print(f"UAV {uav_id:2d}: Position ({uav.x:6.1f}, {uav.y:6.1f}) | Radius: {uav.radius:4.1f}")
        
        # Calculate and display metrics
        if len(self.current_uavs) >= 2:
            self.display_metrics()

    def display_metrics(self):
        """Calculate and display coverage metrics"""
        uav_list = list(self.current_uavs.values())
        distances = []
        intersections = 0
        
        # Calculate distances and intersections
        for i, uav1 in enumerate(uav_list):
            for uav2 in uav_list[i+1:]:
                distance = math.sqrt((uav1.x - uav2.x)**2 + (uav1.y - uav2.y)**2)
                distances.append(distance)
                
                if distance < (uav1.radius + uav2.radius):
                    intersections += 1
        
        avg_distance = sum(distances) / len(distances)
        min_distance = min(distances)
        max_distance = max(distances)
        
        print("\n📈 COVERAGE METRICS:")
        print(f"   Average distance: {avg_distance:6.1f}")
        print(f"   Minimum distance: {min_distance:6.1f}")
        print(f"   Maximum distance: {max_distance:6.1f}")
        print(f"   Intersections:    {intersections:6d}")
        
        # Coverage quality indicator
        if intersections == 0:
            print("   Quality: ✅ EXCELLENT (No overlaps)")
        elif intersections <= 2:
            print("   Quality: 🟡 GOOD (Few overlaps)")
        else:
            print("   Quality: 🔴 POOR (Many overlaps)")

    def draw_ascii_map(self):
        """Simple ASCII visualization"""
        if not self.current_uavs:
            return
        
        width, height = 50, 25
        scale_x = 500 / width
        scale_y = 500 / height
        
        # Create map
        map_grid = [['.' for _ in range(width)] for _ in range(height)]
        
        # Place UAVs
        for uav_id, uav in self.current_uavs.items():
            x_pos = int(uav.x / scale_x)
            y_pos = int(uav.y / scale_y)
            
            x_pos = max(0, min(width - 1, x_pos))
            y_pos = max(0, min(height - 1, y_pos))
            
            map_grid[y_pos][x_pos] = str(uav_id % 10)
        
        print("\n🗺️  ASCII MAP:")
        for row in map_grid:
            print(''.join(row))
        print(f"Scale: 1 char = {scale_x:.1f}x{scale_y:.1f} pixels")


def main(args=None):
    rclpy.init(args=args)
    node = Task2Visualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()