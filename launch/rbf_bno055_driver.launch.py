import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rbf_bno055_driver'),
        'config',
        'rbf_bno055_driver.param.yaml'
    )

    container = ComposableNodeContainer(
    name='rbf_bno055_driver',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='rbf_bno055_driver',
            plugin='rbf_bno055_driver::BNO055Driver',
            name='rbf_bno055_driver_node',
            parameters=[config],
            extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )

    return launch.LaunchDescription([container])