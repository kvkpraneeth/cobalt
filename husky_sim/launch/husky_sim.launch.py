#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path):

    doc = xacro.parse(open(xacro_file_path))
    
    xacro.process_doc(doc)

    return doc

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    xacro_path = os.path.join(
        get_package_share_directory('husky_sim'), 
        'urdf', 
        'husky.sim.xacro'
    )

    world_sdf_path = '-r '

    world_sdf_path += os.path.join(
        get_package_share_directory('cobalt_utils'), 
        'worlds', 
        'empty.sdf'
    )

    robot_sdf_path = os.path.join(
        get_package_share_directory('husky_sim'),
        'models',
        'husky',
        'model.sdf'
    )

    doc = get_xacro_to_doc(xacro_path)

    robot_state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher', 
        parameters=[
            {'use_sim_time': use_sim_time}, 
            {'robot_description': doc.toxml()}
        ],
       remappings=[
            ('joint_states', 'husky/joint_states')
        ]
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
        '-file', robot_sdf_path,
        '-name', 'husky',
        '-allow_renaming', 'true'
        ],
    )

    bridge = Node(
        package='ros_ign_bridge', 
        executable='parameter_bridge', 
        arguments=[
        'husky/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        'husky/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        'husky/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            ('husky/cmd_vel', 'husky/cmd_vel'),
            ('husky/odom','husky/odom'),
            ('husky/joint_states', 'husky/joint_states'),
        ],
    )

    odomTotransform = Node(
        package='cobalt_utils',
        executable='odomTotransform',
        parameters=[
            {'OdometryTopic': 'husky/odom'},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('husky_sim'), 
            'rviz', 
            'config.rviz'
            )
        ],
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),

            launch_arguments=[('ign_args', [world_sdf_path])]),

        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),

        DeclareLaunchArgument('robot_description', default_value=doc.toxml()),

        robot_state_publisher,

        ignition_spawn_entity,

        bridge,

        odomTotransform,

        rviz,

    ])