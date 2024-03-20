import os
import launch_ros
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler
)
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.event_handlers.on_execution_complete import OnExecutionComplete
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    self_pkg = get_package_share_directory('simulation_bringup')

    rviz_config_path = os.path.join(self_pkg, 'config', 'rviz', 'multiple_sim.rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )


    description_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("anymal_c_config"),
                "launch",
                "gazebo.launch.py",
            )
        ),
    )


    map_to_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0', '--qy', '0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'map', '--child-frame-id', 'odom'
            ]
        )
    
    base_to_lidar_frame = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.2',
                '--qx', '0', '--qy', '0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base', '--child-frame-id', 'lidar_frame'
            ]
        )
    

    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time' : LaunchConfiguration("use_sim_time")}],
        arguments=['-d', rviz_config_path],
    )


    initial_pose_pub = Node(
            package='simulation_bringup', executable='initial_pose.py',     
    )

    groundtruth = Node(
            package='simulation_bringup', executable='odom_to_path.py',   
            parameters=[{        
                'frame' : 'odom',
                'odom_topic' : '/odom/ground_truth',
                'path_topic' : '/path',
                # 'filepath' : f'/home/elechim/RF2O_SIM_WS/src/anymal_c_gazebo_sim/simulation_bringup/data/10Hz/GT.txt'
            }]         
    )


    to_launch = [
        declare_use_sim_time,
        description_ld,
        rviz2,
        groundtruth,
        initial_pose_pub,
        map_to_odom,
        base_to_lidar_frame,
    ]

    max_ranges = [5, 10, 15, 20]
    samples = [180, 360]


    for range in max_ranges:

        for sample in samples:

            rf2o_node = Node(
                    package='rf2o',
                    executable='rf2o_laser_odometry_node',
                    output='screen',
                    parameters=[{
                        'laser_scan_topic' : f'/laser_controller_{range}_{sample}/out',
                        'odom_topic' : f'/odom_rf2o_{range}_{sample}',
                        'publish_tf' : True if (sample == 360 and range == 20) else False,
                        'base_frame_id' : 'lidar_frame',
                        'odom_frame_id' : 'odom',
                        'init_pose_from_topic' : 'init_pose',
                        'freq' : 30.0}],
            )

            trajectory_node = Node(
                    package='simulation_bringup', executable='odom_to_path.py',   
                    parameters=[{        
                        'frame' : 'odom',
                        'odom_topic' : f'/odom_rf2o_{range}_{sample}',
                        'path_topic' : f'/path_rf2o_{range}_{sample}',
                        # 'filepath' : f'/home/elechim/RF2O_SIM_WS/src/anymal_c_gazebo_sim/simulation_bringup/data/10Hz/{range}-{sample}.txt'
                    }]         
            )

            to_launch.append(rf2o_node)
            to_launch.append(trajectory_node)


    


    return LaunchDescription(to_launch)

    
