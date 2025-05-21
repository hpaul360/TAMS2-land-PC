import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch.conditions
from ament_index_python.packages import get_package_share_directory
import subprocess

def generate_launch_description():
    log_level = "warn"
    node_1_enabled = LaunchConfiguration('cam')
    node_2_enabled = LaunchConfiguration('arm')

    config = os.path.join(
        get_package_share_directory('tams2_land'),
        'config',
        'tams2_params.yaml'
        )

    # Start MicroXRCEAgent subprocess
    subprocess.Popen(['MicroXRCEAgent', 'udp4', '-p', '8888'])

    # Start QGroundControl subprocess
    subprocess.Popen(['./QGroundControl.AppImage'])

    return launch.LaunchDescription([
        DeclareLaunchArgument('cam', default_value='true', description='Launch landing processor node'),
        DeclareLaunchArgument('arm', default_value='false', description='Launch arms control node'),

        # Start the ROS-Gazebo Bridge
        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/rgb@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/model/tams2_0/servo_0@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/tams2_0/servo_1@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/tams2_0/servo_2@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/tams2_0/servo_3@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/tams2_0/servo_4@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/tams2_0/servo_5@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/tams2_0/servo_6@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            output='screen'
        ),

        # Camera node for landing process
        launch_ros.actions.Node(
            package='tams2_land',
            executable='cam',
            name='tams2_land',
            output='screen',
            parameters=[config],
            condition=launch.conditions.IfCondition(node_1_enabled)
        ),
        
        # Arms control node
        launch_ros.actions.Node(
            package='tams2_land',
            executable='arm',
            name='tams2_land',
            output='screen',
            condition=launch.conditions.IfCondition(node_2_enabled)
        ),
    ])
