import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_file = os.path.join(
        get_package_share_directory('px4_pose'),
        'params',
        'drone_pose.yaml'
    )
    with open(param_file, 'r') as file:
        params = yaml.safe_load(file)

    use_sim_time_param = {'use_sim_time': True}

    return LaunchDescription([
        Node(
            package='px4_pose',
            executable='px4_tf_pub',
            name='px4_pose_node',
            output='screen',
            parameters=[params, use_sim_time_param],
        ),
        Node(
            package='px4_pose',
            executable='px4_drone_control',
            name='px4_control_node',
            output='screen',
            parameters=[use_sim_time_param],
        )
    ])
