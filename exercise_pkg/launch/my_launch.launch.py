from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            # Proper encapsulation of the entire command in single quotes after -e
            prefix='terminator -x sh -c \'ros2 run turtlesim turtle_teleop_key --ros-args -r __node:=teleop\''
        )
    ])
