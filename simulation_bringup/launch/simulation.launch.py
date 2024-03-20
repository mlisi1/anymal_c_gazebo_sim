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

    rviz_config_path = os.path.join(self_pkg, 'config', 'rviz', 'odom_simulation.rviz')

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


    odom_to_path = Node(
            package='simulation_bringup', executable='odom_to_path.py',   
            parameters=[{        
                'frame' : 'odom',
                'odom_topic' : '/odom',
                'path_topic' : '/path'
            }]         
    )
    

    odom_to_path_est_rf2o = Node(
            package='simulation_bringup', executable='odom_to_path.py',   
            parameters=[{        
                'frame' : 'odom',
                'odom_topic' : '/odom_rf2o',
                'path_topic' : '/path_rf2o'
            }]         
    )

    odom_to_path_est_srf = Node(
            package='simulation_bringup', executable='odom_to_path.py',   
            parameters=[{        
                'frame' : 'odom',
                'odom_topic' : '/odom_srf',
                'path_topic' : '/path_srf'
            }]         
    )


    rf2o = Node(
                package='rf2o',
                executable='rf2o_laser_odometry_node',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/laser_controller/out',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : False,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 30.0}],
    )
    

    srf = Node(
                package='srf_laser_odometry',
                executable='srf_laser_odometry_node',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/laser_controller/out',
                    'odom_topic' : '/odom_srf',
                    'publish_tf' : False,
                    'base_frame_id' : 'base',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'operation_mode' : 'CS',
                    }],
    )

    

    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time' : LaunchConfiguration("use_sim_time")}],
        arguments=['-d', rviz_config_path],
    )


    return LaunchDescription(
        [
            declare_use_sim_time,
            description_ld,
            rviz2,
            rf2o,
            odom_to_path,
            odom_to_path_est_rf2o,
        ]
    )