#!/usr/bin/env python3
"""
Spawn obstacles in Gazebo for testing obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import time

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        
        # Wait for Gazebo service
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        
        self.get_logger().info('Obstacle Spawner ready!')
    
    def create_cylinder_sdf(self, radius=0.2, height=1.0):
        """Create SDF model for a cylinder obstacle"""
        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='obstacle'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>1 0 0 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    
    def create_box_sdf(self, size=0.3, height=1.0):
        """Create SDF model for a box obstacle"""
        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='obstacle'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{size} {size} {height}</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{size} {size} {height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    
    def spawn_obstacle(self, name, x, y, z=0.5, obstacle_type='cylinder', size=0.2):
        """Spawn an obstacle at specified position"""
        request = SpawnEntity.Request()
        request.name = name
        
        if obstacle_type == 'cylinder':
            request.xml = self.create_cylinder_sdf(radius=size)
        else:
            request.xml = self.create_box_sdf(size=size)
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        request.initial_pose = pose
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            self.get_logger().info(f'Spawned obstacle: {name} at ({x:.2f}, {y:.2f})')
            return True
        else:
            self.get_logger().error(f'Failed to spawn obstacle: {name}')
            return False
    
    def spawn_test_obstacles(self, pattern='line'):
        """Spawn obstacles in predefined patterns"""
        
        if pattern == 'line':
            # Line of obstacles across path
            self.get_logger().info('Spawning line of obstacles...')
            obstacles = [
                ('obstacle_1', 1.0, 0.5, 0.15),
                ('obstacle_2', 1.5, 0.8, 0.15),
                ('obstacle_3', 2.0, 1.2, 0.15),
            ]
        
        elif pattern == 'corridor':
            # Create a corridor
            self.get_logger().info('Spawning corridor obstacles...')
            obstacles = [
                ('obstacle_1', 1.0, 1.5, 0.15),
                ('obstacle_2', 1.0, -1.5, 0.15),
                ('obstacle_3', 2.5, 1.5, 0.15),
                ('obstacle_4', 2.5, -1.5, 0.15),
            ]
        
        elif pattern == 'scattered':
            # Scattered obstacles in figure-8 path
            self.get_logger().info('Spawning scattered obstacles...')
            obstacles = [
                ('obstacle_1', 1.5, 0.5, 0.2),
                ('obstacle_2', 2.5, 1.0, 0.15),
                ('obstacle_3', 3.5, 0.8, 0.18),
                ('obstacle_4', 2.0, -0.8, 0.15),
                ('obstacle_5', 3.0, -1.2, 0.2),
            ]
        
        elif pattern == 'challenge':
            # Challenging obstacle course
            self.get_logger().info('Spawning challenging obstacles...')
            obstacles = [
                ('obstacle_1', 1.2, 0.3, 0.15),
                ('obstacle_2', 1.8, 1.0, 0.2),
                ('obstacle_3', 2.5, 1.5, 0.15),
                ('obstacle_4', 3.2, 1.3, 0.18),
                ('obstacle_5', 3.8, 0.5, 0.15),
                ('obstacle_6', 3.5, -0.5, 0.2),
                ('obstacle_7', 2.8, -1.2, 0.15),
                ('obstacle_8', 2.0, -1.5, 0.18),
            ]
        
        else:
            # Single test obstacle
            self.get_logger().info('Spawning single test obstacle...')
            obstacles = [
                ('obstacle_1', 2.0, 0.5, 0.2),
            ]
        
        # Spawn all obstacles
        for obs in obstacles:
            name, x, y, size = obs
            self.spawn_obstacle(name, x, y, obstacle_type='cylinder', size=size)
            time.sleep(0.5)
        
        self.get_logger().info(f'Spawned {len(obstacles)} obstacles!')
    
    def clear_obstacles(self):
        """Remove all spawned obstacles"""
        from gazebo_msgs.srv import DeleteEntity
        
        delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        if not delete_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Delete service not available')
            return
        
        # Try to delete common obstacle names
        for i in range(1, 20):
            request = DeleteEntity.Request()
            request.name = f'obstacle_{i}'
            future = delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        self.get_logger().info('Cleared obstacles')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    
    # Get pattern from command line or use default
    import sys
    pattern = sys.argv[1] if len(sys.argv) > 1 else 'scattered'
    
    print("\n" + "="*60)
    print("Obstacle Spawner for Trajectory Control")
    print("="*60)
    print(f"\nSpawning pattern: {pattern}")
    print("\nAvailable patterns:")
    print("  - single: One test obstacle")
    print("  - line: Line of obstacles")
    print("  - corridor: Corridor pattern")
    print("  - scattered: Scattered in figure-8 (DEFAULT)")
    print("  - challenge: Challenging obstacle course")
    print("  - clear: Remove all obstacles")
    print("="*60 + "\n")
    
    if pattern == 'clear':
        node.clear_obstacles()
    else:
        # Wait a bit for Gazebo to be ready
        time.sleep(2.0)
        node.spawn_test_obstacles(pattern)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()