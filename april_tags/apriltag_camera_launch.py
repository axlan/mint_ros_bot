#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1920',
        description='Image width'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='1080',
        description='Image height'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate'
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file:///home/jdiamond/src/turtle_bot/camera/ost.yaml',
        description='Camera calibration info'
    )

    # USB camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'pixel_format': 'mjpeg2rgb',
            'frame_id': 'camera_link',
            'camera_name': 'test_logi',
            'io_method': 'mmap'
        }],
        remappings=[
            ('image_raw', 'camera/image_raw'),
            ('camera_info', 'camera/camera_info')
        ]
    )

    # AprilTag detection node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[{
            'image_transport': 'raw',
            'family': '36h11',
            'size': 0.074,  # Tag size in meters (just black portion)
            'detector': {
                'threads': 4,
            },
        }],
        remappings=[
            ('image_rect', 'camera/image_raw'),
            ('camera_info', 'camera/camera_info')
        ]
    )

    return LaunchDescription([
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        camera_info_url_arg,
        usb_cam_node,
        apriltag_node
    ])
