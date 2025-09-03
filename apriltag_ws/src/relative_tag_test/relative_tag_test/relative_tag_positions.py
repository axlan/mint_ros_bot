#!/usr/bin/env python3

import re
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation
import math

_TAG_RE=re.compile(r'tag.*[:_]([0-9]+)')

class TFAprilTagRelative(Node):
    def __init__(self):
        super().__init__('tf_apriltag_relative')

        # Subscribe to /tf topic
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        # Storage for AprilTag transforms
        self.tag_transforms = {}  # {tag_id: TransformStamped}
        self.reference_tag_id = 0
        self.last_print_time = self.get_clock().now()
        self.print_interval = 2.0  # Print every 2 seconds

        self.get_logger().info('TF AprilTag Relative Position Node started')
        self.get_logger().info(f'Listening to /tf topic for AprilTag transforms')
        self.get_logger().info(f'Reference tag: {self.reference_tag_id}')

    def tf_callback(self, msg):
        """Process incoming TF messages"""
        current_time = self.get_clock().now()

        for transform in msg.transforms:
            # Check if this is an AprilTag transform
            child_frame = transform.child_frame_id

            m = _TAG_RE.match(child_frame)
            if m is not None:
                tag_id = int(m.group(1))
                # Store the transform with timestamp
                self.tag_transforms[tag_id] = transform
                self.get_logger().info(tag_id)

        # Print relative positions periodically
        time_diff = (current_time - self.last_print_time).nanoseconds / 1e9
        if time_diff >= self.print_interval:
            self.print_relative_positions()
            self.last_print_time = current_time

    def transform_to_matrix(self, transform_stamped):
        """Convert ROS TransformStamped to 4x4 transformation matrix"""
        t = transform_stamped.transform.translation
        r = transform_stamped.transform.rotation

        # Convert quaternion to rotation matrix
        rotation = Rotation.from_quat([r.x, r.y, r.z, r.w])
        rot_matrix = rotation.as_matrix()

        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = [t.x, t.y, t.z]

        return T

    def matrix_to_position_orientation(self, matrix):
        """Extract position and orientation from transformation matrix"""
        position = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        rotation = Rotation.from_matrix(rotation_matrix)
        euler = rotation.as_euler('xyz', degrees=True)  # Roll, Pitch, Yaw in degrees

        return position, euler

    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions"""
        return np.linalg.norm(pos1 - pos2)

    def print_relative_positions(self):
        """Calculate and print positions of all tags relative to reference tag"""
        if not self.tag_transforms:
            return

        # Check if reference tag exists
        if self.reference_tag_id not in self.tag_transforms:
            available_tags = list(self.tag_transforms.keys())
            if available_tags:
                self.get_logger().warn(f'Reference tag {self.reference_tag_id} not found. '
                                     f'Available tags: {sorted(available_tags)}')
            return

        # Get reference tag transform
        ref_transform = self.tag_transforms[self.reference_tag_id]
        ref_matrix = self.transform_to_matrix(ref_transform)
        ref_matrix_inv = np.linalg.inv(ref_matrix)

        # Get reference position for distance calculations
        ref_pos, _ = self.matrix_to_position_orientation(ref_matrix)

        self.get_logger().info('=' * 80)
        self.get_logger().info(f'AprilTag positions relative to tag {self.reference_tag_id}:')
        self.get_logger().info(f'Found {len(self.tag_transforms)} tags: {sorted(self.tag_transforms.keys())}')
        self.get_logger().info('=' * 80)

        # Sort tags by ID for consistent output
        sorted_tags = sorted(self.tag_transforms.keys())

        for tag_id in sorted_tags:
            if tag_id == self.reference_tag_id:
                # Reference tag is at origin
                self.get_logger().info(
                    f'Tag {tag_id:2d} (ref): pos=(0.000, 0.000, 0.000) | '
                    f'rot=(  0.0°,   0.0°,   0.0°) | dist=0.000m'
                )
                continue

            # Get current tag transform
            tag_transform = self.tag_transforms[tag_id]
            tag_matrix = self.transform_to_matrix(tag_transform)

            # Calculate relative transform: T_rel = T_ref^-1 * T_tag
            relative_matrix = ref_matrix_inv @ tag_matrix

            # Extract position and orientation
            rel_pos, rel_euler = self.matrix_to_position_orientation(relative_matrix)

            # Calculate distance from reference tag
            tag_pos, _ = self.matrix_to_position_orientation(tag_matrix)
            distance = self.calculate_distance(ref_pos, tag_pos)

            self.get_logger().info(
                f'Tag {tag_id:2d}:      pos=({rel_pos[0]:6.3f}, {rel_pos[1]:6.3f}, {rel_pos[2]:6.3f}) | '
                f'rot=({rel_euler[0]:5.1f}°, {rel_euler[1]:5.1f}°, {rel_euler[2]:5.1f}°) | '
                f'dist={distance:.3f}m'
            )

        # Print timestamp info
        current_time = self.get_clock().now()
        self.get_logger().info(f'Last update: {current_time.nanoseconds / 1e9:.1f}s')
        self.get_logger().info('')

    def print_transform_details(self, tag_id, transform):
        """Print detailed transform information for debugging"""
        t = transform.transform.translation
        r = transform.transform.rotation

        self.get_logger().info(f'Tag {tag_id} detailed transform:')
        self.get_logger().info(f'  Frame: {transform.header.frame_id} -> {transform.child_frame_id}')
        self.get_logger().info(f'  Translation: x={t.x:.6f}, y={t.y:.6f}, z={t.z:.6f}')
        self.get_logger().info(f'  Rotation (quat): x={r.x:.6f}, y={r.y:.6f}, z={r.z:.6f}, w={r.w:.6f}')

        # Convert to Euler angles
        rotation = Rotation.from_quat([r.x, r.y, r.z, r.w])
        euler = rotation.as_euler('xyz', degrees=True)
        self.get_logger().info(f'  Rotation (euler): roll={euler[0]:.2f}°, pitch={euler[1]:.2f}°, yaw={euler[2]:.2f}°')


def main(args=None):
    rclpy.init(args=args)

    node = TFAprilTagRelative()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
