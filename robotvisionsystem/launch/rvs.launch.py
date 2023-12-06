from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ros_tcp_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "127.0.0.1"}, {"ROS_TCP_PORT": 10000}],
    )
    # main_node = Node(
    #     package="robotvisionsystem",
    #     executable="main",
    #     output="screen"
    # )

    return LaunchDescription([
        ros_tcp_node,
        # main_node,s
    ])
