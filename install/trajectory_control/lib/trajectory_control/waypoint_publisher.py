#!/usr/bin/env python3
"""
Extended waypoint publisher with longer, more complex trajectories
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import math

class ExtendedWaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        self.publisher = self.create_publisher(PoseArray, 'waypoints', 10)
        
        # Declare parameters
        self.declare_parameter('pattern', 'extended_figure8')
        self.declare_parameter('auto_publish', True)
        self.declare_parameter('delay', 2.0)
        
        # Get parameters
        self.pattern = self.get_parameter('pattern').value
        auto_publish = self.get_parameter('auto_publish').value
        delay = self.get_parameter('delay').value
        
        self.published = False
        
        if auto_publish:
            # Timer to publish waypoints after delay
            self.timer = self.create_timer(delay, self.publish_waypoints)
        
        self.get_logger().info(f'Waypoint Publisher initialized')
        self.get_logger().info(f'Pattern: {self.pattern}')
        self.get_logger().info(f'Available patterns: extended_figure8, large_loop, spiral, zigzag, race_track, exploration')
    
    def publish_waypoints(self):
        if self.published:
            return
        
        pose_array = PoseArray()
        pose_array.header.frame_id = 'odom'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        # Generate waypoints based on pattern
        if self.pattern == 'extended_figure8':
            waypoints = self.generate_extended_figure8()
        elif self.pattern == 'large_loop':
            waypoints = self.generate_large_loop()
        elif self.pattern == 'spiral':
            waypoints = self.generate_spiral()
        elif self.pattern == 'zigzag':
            waypoints = self.generate_zigzag()
        elif self.pattern == 'race_track':
            waypoints = self.generate_race_track()
        elif self.pattern == 'exploration':
            waypoints = self.generate_exploration()
        elif self.pattern == 'figure8':
            waypoints = self.generate_figure8()  # Original
        elif self.pattern == 'square':
            waypoints = self.generate_square()
        elif self.pattern == 'circle':
            waypoints = self.generate_circle()
        else:
            waypoints = self.generate_extended_figure8()
        
        for x, y in waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.publisher.publish(pose_array)
        self.get_logger().info(f'Published {len(waypoints)} waypoints ({self.pattern} pattern)')
        self.get_logger().info(f'Path length: ~{self.estimate_path_length(waypoints):.1f} meters')
        self.published = True
    
    def estimate_path_length(self, waypoints):
        """Estimate total path length"""
        length = 0.0
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1][0] - waypoints[i][0]
            dy = waypoints[i+1][1] - waypoints[i][1]
            length += math.sqrt(dx**2 + dy**2)
        return length
    
    def generate_extended_figure8(self):
        """Generate extended figure-8 pattern with more waypoints"""
        waypoints = []
        num_points = 25  # Increased from 13
        
        for i in range(num_points + 1):
            t = i * 2 * math.pi / num_points
            # Larger figure-8
            x = 3.0 + 2.5 * math.cos(t)
            y = 1.5 * math.sin(t)
            waypoints.append((x, y))
        
        return waypoints
    
    def generate_large_loop(self):
        """Generate large circular loop"""
        waypoints = []
        num_points = 32
        radius = 3.0
        center_x, center_y = 3.0, 0.0
        
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            waypoints.append((x, y))
        
        return waypoints
    
    def generate_spiral(self):
        """Generate spiral pattern moving outward"""
        waypoints = []
        num_loops = 3
        points_per_loop = 16
        max_radius = 3.0
        center_x, center_y = 2.0, 0.0
        
        total_points = num_loops * points_per_loop
        
        for i in range(total_points + 1):
            theta = 2 * math.pi * i / points_per_loop
            radius = max_radius * (i / total_points)
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            waypoints.append((x, y))
        
        return waypoints
    
    def generate_zigzag(self):
        """Generate zigzag pattern"""
        waypoints = []
        num_segments = 8
        segment_length = 1.5
        amplitude = 1.0
        
        for i in range(num_segments + 1):
            x = i * segment_length
            y = amplitude * (1 if i % 2 == 0 else -1)
            waypoints.append((x, y))
        
        return waypoints
    
    def generate_race_track(self):
        """Generate race track with straights and curves"""
        waypoints = []
        
        # Start straight
        for i in range(5):
            waypoints.append((i * 0.5, 0.0))
        
        # First curve (right turn)
        for i in range(8):
            angle = i * math.pi / 14
            waypoints.append((
                2.0 + 1.5 * math.cos(angle),
                1.5 * math.sin(angle)
            ))
        
        # Top straight
        for i in range(5):
            waypoints.append((3.5 - i * 0.5, 1.5))
        
        # Second curve (right turn)
        for i in range(8):
            angle = math.pi / 2 + i * math.pi / 14
            waypoints.append((
                0.5 + 1.5 * math.cos(angle),
                1.5 + 1.5 * math.sin(angle)
            ))
        
        # Left straight (return)
        for i in range(5):
            waypoints.append((0.0, 1.5 - i * 0.3))
        
        # Close to start
        waypoints.append((0.0, 0.0))
        
        return waypoints
    
    def generate_exploration(self):
        """Generate exploration pattern covering large area"""
        waypoints = [
            (0.0, 0.0),
            (1.5, 0.0),
            (1.5, 1.5),
            (3.0, 1.5),
            (3.0, 3.0),
            (4.5, 3.0),
            (4.5, 1.5),
            (6.0, 1.5),
            (6.0, 0.0),
            (4.5, 0.0),
            (4.5, -1.5),
            (3.0, -1.5),
            (3.0, -3.0),
            (1.5, -3.0),
            (1.5, -1.5),
            (0.0, -1.5),
            (0.0, 0.0),
        ]
        return waypoints
    
    # Original patterns for compatibility
    def generate_figure8(self):
        """Original figure-8 pattern"""
        waypoints = []
        for i in range(13):
            t = i * 2 * math.pi / 12
            x = 2.0 + 1.5 * math.cos(t)
            y = 1.0 * math.sin(t)
            waypoints.append((x, y))
        return waypoints
    
    def generate_square(self):
        """Generate square pattern"""
        return [
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 3.0),
            (0.0, 3.0),
            (0.0, 0.0)
        ]
    
    def generate_circle(self):
        """Generate circular pattern"""
        waypoints = []
        num_points = 24
        radius = 2.5
        center_x, center_y = 2.5, 0.0
        
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            waypoints.append((x, y))
        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = ExtendedWaypointPublisher()
    
    print("\n" + "="*70)
    print("Extended Waypoint Publisher for Trajectory Control")
    print("="*70)
    print("\nAvailable trajectory patterns:")
    print("  • extended_figure8  - Larger figure-8 with 25 waypoints (DEFAULT)")
    print("  • large_loop        - Large circular path (3m radius)")
    print("  • spiral            - Spiral pattern expanding outward")
    print("  • zigzag            - Zigzag pattern across space")
    print("  • race_track        - Race track with straights and curves")
    print("  • exploration       - Exploration pattern covering large area")
    print("  • figure8           - Original figure-8 (13 waypoints)")
    print("  • square            - Simple square path")
    print("  • circle            - Circular path")
    print("\nUsage:")
    print("  ros2 run trajectory_control waypoint_publisher.py")
    print("  ros2 run trajectory_control waypoint_publisher.py --ros-args -p pattern:=spiral")
    print("="*70 + "\n")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()