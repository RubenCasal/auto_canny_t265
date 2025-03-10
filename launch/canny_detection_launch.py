import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Start the T265 node (C++ executable)
        launch_ros.actions.Node(
            package='auto_canny_t265',
            executable='t265_node',
            name='t265_node',
            output='screen'
        ),

        # Start the Auto Canny Node (Python script)
        launch_ros.actions.Node(
            package='auto_canny_t265',
            executable='auto_canny.py',  # Ensure script name matches
            name='auto_canny_node',
            output='screen',
            parameters=[],
            arguments=[os.path.join(os.getenv('ROS_WS', '/home/rcasal/ros2_ws'), 'install/opencv_t265/lib/opencv_t265/auto_canny.py')]
        ),
    ])
