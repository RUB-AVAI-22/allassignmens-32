from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch = LaunchDescription()

    camera = Node(package="camera_task", executable="camera")
    image_process = Node(package="camera_task", executable="process")

    launch.add_action(camera)
    launch.add_action(image_process)
    return launch
