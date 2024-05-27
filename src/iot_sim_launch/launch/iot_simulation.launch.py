from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Step 3: Run the rosbag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '../../../data/r2b_hope.db3', '--loop', '--rate', '0.5']
        ),

        # Step 4: Visualize the data in RViz2
        ExecuteProcess(
            cmd=['rviz2', '-d', 'camera_vis.rviz']
        ),

        # Step 5: Run the logger
        Node(
            package='iot_logger',
            executable='iot_logger',
            name='iot_logger',
            output='screen',
        ),

        # Step 6: Run the uploader
        Node(
            package='iot_uploader',
            executable='iot_uploader',
            name='iot_uploader',
            output='screen',
        )
    ])
