#!/usr/bin/env python3
"""
Goal Sender - Easy way to send navigation goals in different frames!

Usage:
    # Relative to drone (10m forward, 5m left, 2m up)
    ./send_goal.py --relative 10 5 2
    
    # Absolute in map frame
    ./send_goal.py --map 10 5 5
    
    # Offset from current position
    ./send_goal.py --offset 5 0 1
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import argparse
import sys


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.publisher = self.create_publisher(PointStamped, '/nav_goal_3d', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry', self.odom_callback, 10)
        self.current_pos = None
        
    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        
    def send_goal(self, x, y, z, frame_id='map'):
        """Send a goal in the specified frame"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Goal sent in frame "{frame_id}": ({x:.2f}, {y:.2f}, {z:.2f})')
        
    def send_relative_goal(self, forward, left, up):
        """Send goal relative to drone (base_link frame)"""
        self.send_goal(forward, left, up, frame_id='base_link')
        
    def send_map_goal(self, x, y, z):
        """Send goal in map coordinates"""
        self.send_goal(x, y, z, frame_id='map')
        
    def send_offset_goal(self, dx, dy, dz):
        """Send goal as offset from current position"""
        if self.current_pos is None:
            self.get_logger().error('No odometry received yet! Waiting...')
            return False
            
        x = self.current_pos.x + dx
        y = self.current_pos.y + dy
        z = self.current_pos.z + dz
        
        self.get_logger().info(
            f'Current position: ({self.current_pos.x:.2f}, '
            f'{self.current_pos.y:.2f}, {self.current_pos.z:.2f})')
        self.send_map_goal(x, y, z)
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Send navigation goals in different coordinate frames')
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--relative', nargs=3, type=float, metavar=('FORWARD', 'LEFT', 'UP'),
                      help='Goal relative to drone (base_link frame). '
                           'Example: --relative 10 5 2 means 10m forward, 5m left, 2m up from drone')
    group.add_argument('--map', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                      help='Goal in map frame (absolute coordinates). '
                           'Example: --map 10 5 5')
    group.add_argument('--offset', nargs=3, type=float, metavar=('DX', 'DY', 'DZ'),
                      help='Goal as offset from current position in map frame. '
                           'Example: --offset 5 0 2 means +5m in X, no change in Y, +2m in Z')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = GoalSender()
    
    # Wait for odometry if using offset mode
    if args.offset:
        print('Waiting for odometry...')
        rate = node.create_rate(10)
        for _ in range(50):  # Wait up to 5 seconds
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.current_pos is not None:
                break
        
        if node.current_pos is None:
            print('ERROR: No odometry received after 5 seconds!')
            print('Make sure the drone is running and publishing /odometry')
            rclpy.shutdown()
            return
    
    # Send the goal
    if args.relative:
        forward, left, up = args.relative
        print(f'\n📍 Sending RELATIVE goal:')
        print(f'   Forward: {forward:.2f} m')
        print(f'   Left:    {left:.2f} m')
        print(f'   Up:      {up:.2f} m')
        print(f'   (in base_link frame - relative to drone)')
        node.send_relative_goal(forward, left, up)
        
    elif args.map:
        x, y, z = args.map
        print(f'\n📍 Sending MAP goal:')
        print(f'   X: {x:.2f} m')
        print(f'   Y: {y:.2f} m')
        print(f'   Z: {z:.2f} m')
        print(f'   (in map frame - absolute coordinates)')
        node.send_map_goal(x, y, z)
        
    elif args.offset:
        dx, dy, dz = args.offset
        print(f'\n📍 Sending OFFSET goal:')
        print(f'   +X: {dx:.2f} m')
        print(f'   +Y: {dy:.2f} m')
        print(f'   +Z: {dz:.2f} m')
        print(f'   (offset from current position)')
        if not node.send_offset_goal(dx, dy, dz):
            rclpy.shutdown()
            return
    
    print('\n✓ Goal sent successfully!')
    print('  Check navigator terminal for confirmation.\n')
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()