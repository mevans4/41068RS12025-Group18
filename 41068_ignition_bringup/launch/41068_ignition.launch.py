from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

#Keyboard control
from launch.actions import OpaqueFunction


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Assuming you have your drone URDF (or xacro) inside the sjtu_drone_description package
    drone_pkg_path = FindPackageShare('sjtu_drone_description')

    # Replace 'drone.urdf.xacro' with the actual file name of your drone's URDF or Xacro file
    drone_urdf = PathJoinSubstitution([drone_pkg_path, 'urdf', 'sjtu_drone.urdf.xacro'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)
    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    # # Load robot_description and start robot_state_publisher HUSKY
    # robot_description_content = ParameterValue(
    #     Command(['xacro ',
    #              PathJoinSubstitution([pkg_path,
    #                                    'urdf',
    #                                    'husky.urdf.xacro'])]),
    #     value_type=str)
    # robot_state_publisher_node = Node(package='robot_state_publisher',
    #                                   executable='robot_state_publisher',
    #                                   parameters=[{
    #                                       'robot_description': robot_description_content,
    #                                       'use_sim_time': use_sim_time
    #                                   }])
    # ld.add_action(robot_state_publisher_node)

    # Process the xacro to get the robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', drone_urdf]),
        value_type=str
    )

    # Launch robot_state_publisher node with drone's description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    ld.add_action(robot_state_publisher_node)

    

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    ld.add_action(world_launch_arg)

    # OLD Launch
    # gazebo = IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
    #                          'launch', 'ign_gazebo.launch.py']),
    #     launch_arguments={
    #         'ign_args': [PathJoinSubstitution([pkg_path,
    #                                            'worlds',
    #                                            [LaunchConfiguration('world'), '.sdf']]),
    #                      ' -r']}.items()
    # )

    #Fixed for WSL
    gazebo = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py']),
    launch_arguments={
        'ign_args': [
            PathJoinSubstitution([pkg_path, 'worlds', [LaunchConfiguration('world'), '.sdf']]),
            ' -r --render-engine ogre'
        ]
    }.items()
    )
    ld.add_action(gazebo)

    # # Include the drone's Gazebo simulation and control launch
    # drone_launch = IncludeLaunchDescription(
    #     PathJoinSubstitution([
    #         FindPackageShare('sjtu_drone_bringup'),
    #         'launch',
    #         'sjtu_drone_bringup.launch.py'
    #     ])
    # )
    # ld.add_action(drone_launch)

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-name', 'sjtu_drone','-topic', '/robot_description', '-z', '0.2']
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    return ld
