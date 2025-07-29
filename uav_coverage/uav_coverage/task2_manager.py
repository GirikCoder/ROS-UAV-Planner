#!/usr/bin/env python3
import random
import time
import json
from typing import Dict
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_uav_interfaces.msg import UAVInfo, UAVCollection, OptimizeCommand


class Task2Manager(Node):
    def __init__(self):
        super().__init__('task2_manager')
        
        # Publishers
        self.uav_state_publisher = self.create_publisher(UAVCollection, 'current_uav_states', 10)
        self.optimize_trigger_publisher = self.create_publisher(OptimizeCommand, 'optimize_trigger', 10)
        
        # Subscribers
        self.optimized_positions_sub = self.create_subscription(
            UAVCollection, 'optimized_positions', self.update_positions_callback, 10)
        self.initial_uav_sub = self.create_subscription(
            UAVCollection, 'initial_uav_placement', self.receive_initial_uavs_callback, 10)
        # UAV management
        self.active_uavs: Dict[int, UAVInfo] = {}
        self.next_uav_id = 0
        self.screen_width = 500
        self.screen_height = 500
        self.uav_radius = 50
        self.initial_uavs_received = False
        # Dynamic parameters
        self.birth_rate = 0.15   # 15% chance per second
        self.death_rate = 0.15  # 15% chance per second  
        self.min_uavs = 2
        self.max_uavs = 12
        
        # Initialize with some UAVs
        # self.initialize_uavs(5)
        
        # Timers
        self.birth_death_timer = None
        self.publish_timer = None
        
        self.get_logger().info(" Task 2 UAV Manager started!")
        self.get_logger().info(" Waiting for initial UAVs from Task 1...")

    def receive_initial_uavs_callback(self, msg: UAVCollection):
        """Receive initial UAVs from Task 1"""
        if self.initial_uavs_received:
            return  # Only accept initial UAVs once
        
        self.get_logger().info(f" Received {msg.count} initial UAVs from Task 1")
        
        # Clear any existing UAVs
        self.active_uavs.clear()
        
        # Add the UAVs from Task 1
        for uav in msg.uavs:
            self.active_uavs[uav.id] = uav
            self.get_logger().info(f" Initial UAV {uav.id} at ({uav.x:.0f}, {uav.y:.0f})")
        
        # Update next ID to avoid conflicts
        self.next_uav_id = max(self.active_uavs.keys()) + 1 if self.active_uavs else 0
        
        self.initial_uavs_received = True
        self.get_logger().info(" Task 2 dynamic management started with initial UAVs!")

        # NOW START THE TIMERS
        self.birth_death_timer = self.create_timer(1.0, self.birth_death_cycle)
        self.publish_timer = self.create_timer(0.2, self.publish_current_state)
        self.get_logger().info(" Dynamic birth/death cycle activated!")
        
        # Trigger initial optimization
        self.trigger_optimization()

    def add_random_uav(self):
        """Add new UAV at random position"""
        uav = UAVInfo()
        uav.id = self.next_uav_id
        uav.x = float(random.randint(self.uav_radius, self.screen_width - self.uav_radius))
        uav.y = float(random.randint(self.uav_radius, self.screen_height - self.uav_radius))
        uav.radius = float(self.uav_radius)
        
        self.active_uavs[self.next_uav_id] = uav
        self.get_logger().info(f" UAV {self.next_uav_id} BORN at ({uav.x:.0f}, {uav.y:.0f})")
        self.next_uav_id += 1
        
        return uav

    def remove_uav(self, uav_id: int):
        """Remove UAV from system"""
        if uav_id in self.active_uavs:
            uav = self.active_uavs.pop(uav_id)
            self.get_logger().info(f" UAV {uav_id} DIED at ({uav.x:.0f}, {uav.y:.0f})")
            return True
        return False

    def birth_death_cycle(self):
        """Handle random UAV birth/death events"""
        current_count = len(self.active_uavs)
        
        # Birth event
        if current_count < self.max_uavs and random.random() < self.birth_rate:
            self.add_random_uav()
            self.trigger_optimization()
        
        # Death event  
        elif current_count > self.min_uavs and random.random() < self.death_rate:
            # Choose random UAV to remove
            uav_id = random.choice(list(self.active_uavs.keys()))
            self.remove_uav(uav_id)
            self.trigger_optimization()

    def trigger_optimization(self):
        """Send optimization command"""
        cmd = OptimizeCommand()
        cmd.command = "optimize"
        cmd.trigger_id = int(time.time() * 1000) % 10000  # Simple trigger ID
        
        self.optimize_trigger_publisher.publish(cmd)
        self.get_logger().info(f" Triggered optimization for {len(self.active_uavs)} UAVs")

    def publish_current_state(self):
        """Publish current UAV states"""
        if not self.active_uavs:
            return
        
        collection = UAVCollection()
        collection.count = len(self.active_uavs)
        collection.uavs = list(self.active_uavs.values())
        
        self.uav_state_publisher.publish(collection)

    def update_positions_callback(self, msg: UAVCollection):
        """Update UAV positions from optimizer"""
        updates = 0
        for optimized_uav in msg.uavs:
            if optimized_uav.id in self.active_uavs:
                self.active_uavs[optimized_uav.id].x = optimized_uav.x
                self.active_uavs[optimized_uav.id].y = optimized_uav.y
                updates += 1
        
        if updates > 0:
            self.get_logger().info(f" Updated {updates} UAV positions")


def main(args=None):
    rclpy.init(args=args)
    node = Task2Manager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()