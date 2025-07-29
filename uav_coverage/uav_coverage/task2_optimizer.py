#!/usr/bin/env python3
import math
import numpy as np
from typing import Dict, List
import rclpy
from rclpy.node import Node
from my_uav_interfaces.msg import UAVInfo, UAVCollection, OptimizeCommand


class Task2Optimizer(Node):
    def __init__(self):
        super().__init__('task2_optimizer')
        
        # Subscribers
        self.uav_states_sub = self.create_subscription(
            UAVCollection, 'current_uav_states', self.uav_states_callback, 10)
        self.optimize_trigger_sub = self.create_subscription(
            OptimizeCommand, 'optimize_trigger', self.optimize_callback, 10)
        
        # Publishers
        self.optimized_positions_pub = self.create_publisher(
            UAVCollection, 'optimized_positions', 10)
        
        # Current UAV data
        self.current_uavs: Dict[int, UAVInfo] = {}
        self.workspace_bounds = (500, 500)  # width, height
        self.uav_radius = 50
        
        # Optimization parameters
        self.max_iterations = 30
        self.convergence_threshold = 1.0
        self.force_strength = 50.0
        self.boundary_force = 100.0
        self.step_size = 0.15
        
        self.get_logger().info(" Task 2 Position Optimizer ready!")

    def uav_states_callback(self, msg: UAVCollection):
        """Store current UAV states"""
        self.current_uavs.clear()
        for uav in msg.uavs:
            self.current_uavs[uav.id] = uav

    def optimize_callback(self, msg: OptimizeCommand):
        """Triggered when optimization needed"""
        if len(self.current_uavs) >= 2:
            self.get_logger().info(f"⚡ Starting optimization for {len(self.current_uavs)} UAVs...")
            optimized_positions = self.optimize_positions()
            self.publish_optimized_positions(optimized_positions)
        else:
            self.get_logger().info(" Skipping optimization (< 2 UAVs)")

    def optimize_positions(self) -> Dict[int, UAVInfo]:
        """Optimize UAV positions using force-based algorithm"""
        if len(self.current_uavs) < 2:
            return self.current_uavs.copy()
        
        # Convert to numpy arrays for calculation
        positions = {}
        for uav_id, uav in self.current_uavs.items():
            positions[uav_id] = np.array([uav.x, uav.y], dtype=float)
        
        # Iterative optimization
        for iteration in range(self.max_iterations):
            forces = {uav_id: np.array([0.0, 0.0]) for uav_id in positions.keys()}
            
            # Calculate repulsive/attractive forces between UAVs
            uav_ids = list(positions.keys())
            for i, uav_id1 in enumerate(uav_ids):
                for j, uav_id2 in enumerate(uav_ids[i+1:], i+1):
                    force = self.calculate_interaction_force(
                        positions[uav_id1], positions[uav_id2])
                    forces[uav_id1] += force
                    forces[uav_id2] -= force  # Newton's third law
            
            # Calculate boundary forces
            for uav_id, pos in positions.items():
                boundary_force = self.calculate_boundary_force(pos)
                forces[uav_id] += boundary_force
            
            # Update positions
            max_displacement = 0.0
            new_positions = {}
            for uav_id, pos in positions.items():
                new_pos = pos + self.step_size * forces[uav_id]
                new_pos = self.clamp_to_bounds(new_pos)
                
                displacement = np.linalg.norm(new_pos - pos)
                max_displacement = max(max_displacement, displacement)
                new_positions[uav_id] = new_pos
            
            positions = new_positions
            
            # Check convergence
            if max_displacement < self.convergence_threshold:
                self.get_logger().info(f" Converged after {iteration + 1} iterations")
                break
        
        # Convert back to UAVInfo objects
        result = {}
        for uav_id, pos in positions.items():
            uav = UAVInfo()
            uav.id = uav_id
            uav.x = float(pos[0])
            uav.y = float(pos[1])
            uav.radius = float(self.uav_radius)
            result[uav_id] = uav
        
        return result

    def calculate_interaction_force(self, pos1: np.ndarray, pos2: np.ndarray) -> np.ndarray:
        """Calculate force between two UAVs"""
        diff = pos1 - pos2
        distance = np.linalg.norm(diff)
        
        if distance < 1e-6:  # Avoid division by zero
            return np.random.randn(2) * self.force_strength
        
        # Optimal distance: sum of radii + buffer
        optimal_distance = 2 * self.uav_radius + 30  # 30 pixel buffer
        
        if distance < optimal_distance:
            # Repulsion when too close
            force_magnitude = self.force_strength * (optimal_distance - distance) / distance
            return force_magnitude * (diff / distance)
        else:
            # Weak attraction when too far apart
            force_magnitude = -0.05 * self.force_strength * (distance - optimal_distance) / distance
            return force_magnitude * (diff / distance)

    def calculate_boundary_force(self, pos: np.ndarray) -> np.ndarray:
        """Calculate force to keep UAV within bounds"""
        force = np.array([0.0, 0.0])
        margin = self.uav_radius
        
        # Left boundary
        if pos[0] < margin:
            force[0] += self.boundary_force * (margin - pos[0])
        
        # Right boundary
        if pos[0] > self.workspace_bounds[0] - margin:
            force[0] -= self.boundary_force * (pos[0] - (self.workspace_bounds[0] - margin))
        
        # Top boundary
        if pos[1] < margin:
            force[1] += self.boundary_force * (margin - pos[1])
        
        # Bottom boundary
        if pos[1] > self.workspace_bounds[1] - margin:
            force[1] -= self.boundary_force * (pos[1] - (self.workspace_bounds[1] - margin))
        
        return force

    def clamp_to_bounds(self, pos: np.ndarray) -> np.ndarray:
        """Ensure position stays within workspace"""
        margin = self.uav_radius
        clamped = pos.copy()
        clamped[0] = np.clip(clamped[0], margin, self.workspace_bounds[0] - margin)
        clamped[1] = np.clip(clamped[1], margin, self.workspace_bounds[1] - margin)
        return clamped

    def publish_optimized_positions(self, optimized_uavs: Dict[int, UAVInfo]):
        """Publish optimized positions"""
        collection = UAVCollection()
        collection.count = len(optimized_uavs)
        collection.uavs = list(optimized_uavs.values())
        
        self.optimized_positions_pub.publish(collection)
        self.get_logger().info(f" Published optimized positions for {len(optimized_uavs)} UAVs")


def main(args=None):
    rclpy.init(args=args)
    node = Task2Optimizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()