import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():   
    package_dir = get_package_share_directory('lynxmotion_care')
    agent_args = ["serial", "--dev", "/dev/ttyUSB0"]
    teleop = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                package_dir + '/launch/teleop.launch.py'))

    return launch.LaunchDescription([
    #     launch_ros.actions.Node(
    #         package='lynxmotion_care',
    #         executable='hw_serial.py',
    #         name='hw_serial_node',
    #         output="screen",
    #     ),
        launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=agent_args,
            output="screen",
        ),
        teleop,
        
    ])
