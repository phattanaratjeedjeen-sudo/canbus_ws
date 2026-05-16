from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'],
            prefix='gnome-terminal --'
        ),
        Node(
            package='hardware_control',
            executable='motor_control.py',
            output='screen',
            prefix='gnome-terminal --'
        )
    ])