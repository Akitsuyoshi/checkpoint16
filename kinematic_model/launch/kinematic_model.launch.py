from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wheel_vel_pub = Node(
        package="wheel_velocities_publisher",
        executable="wheel_velocities_publisher",
        name="wheel_velocities_publisher",
        output="screen"
    )

    kinematic_model = Node(
        package="kinematic_model",
        executable="kinematic_model",
        name="kinematic_model",
        output="screen"
    )

    return LaunchDescription([
        wheel_vel_pub,
        kinematic_model,
    ])
