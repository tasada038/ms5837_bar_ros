from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "ms5837_bar_ros"
    rviz_file_name = "bar02.rviz"

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "rviz", rviz_file_name]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    ping1d_node = Node(
        package='ms5837_bar_ros',
        executable='bar02_node',
        output="screen",
    )

    base_to_range = Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'bar02_link']
    )

    nodes = [
        rviz_node,
        ping1d_node,
        base_to_range,
    ]

    return LaunchDescription(nodes)
