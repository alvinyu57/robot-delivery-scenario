"""Launch the delivery room, robot A, delivery area B, and police P."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create Gazebo, spawn both Xacro models, and bridge robot topics."""
    package_share = Path(
        get_package_share_directory('delivery_robot_description')
    )
    world_path = package_share / 'worlds' / 'world.sdf'
    gazebo_gui_config = package_share / 'config' / 'gazebo_gui.config'
    rviz_config = package_share / 'rviz' / 'navigation.rviz'
    robot_xacro = (
        package_share / 'models' / 'delivery_robot'
        / 'delivery_robot.urdf.xacro'
    )
    police_xacro = (
        package_share / 'models' / 'traffic_police'
        / 'traffic_police.urdf.xacro'
    )
    navigation_launch = (
        Path(get_package_share_directory('delivery_robot'))
        / 'launch' / 'navigation.launch.py'
    )

    gz_launch = (
        Path(get_package_share_directory('ros_gz_sim'))
        / 'launch' / 'gz_sim.launch.py'
    )
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        condition=IfCondition(gui),
        launch_arguments={
            'gz_args': (
                f'-r -v 3 --gui-config {gazebo_gui_config} {world_path}'
            ),
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        condition=UnlessCondition(gui),
        launch_arguments={
            'gz_args': f'-r -s -v 3 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_description = ParameterValue(
        Command(['xacro ', str(robot_xacro)]),
        value_type=str,
    )
    police_description = ParameterValue(
        Command(['xacro ', str(police_xacro)]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}
        ],
    )
    police_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='traffic_police',
        parameters=[
            {'robot_description': police_description, 'use_sim_time': True}
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'robot_a',
            '-x', '-8.0', '-y', '0.0', '-z', '0.05',
        ],
        output='screen',
    )
    spawn_police = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/traffic_police/robot_description',
            '-name', 'traffic_police_p',
            '-x', '1.25', '-y', '0.75', '-z', '0.0',
            '-Y', '3.141593',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='delivery_description_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            (
                '/delivery_robot/rgbd/image'
                '@sensor_msgs/msg/Image[gz.msgs.Image'
            ),
            (
                '/delivery_robot/rgbd/camera_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ),
            (
                '/delivery_robot/rgbd/depth_image'
                '@sensor_msgs/msg/Image[gz.msgs.Image'
            ),
            '/delivery_robot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/delivery_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            (
                '/delivery_robot/cmd_vel'
                '@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ),
            (
                '/world/delivery_scenario/model/robot_a/joint_state'
                '@sensor_msgs/msg/JointState[gz.msgs.Model'
            ),
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/delivery_robot/tf', '/tf'),
            (
                '/world/delivery_scenario/model/robot_a/joint_state',
                '/joint_states',
            ),
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(navigation_launch)),
    )
    traffic_police = Node(
        package='traffic_police',
        executable='traffic_police_node',
        name='traffic_police_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config)],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value=gui,
            description='Start RViz2 with the navigation view.',
        ),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        police_state_publisher,
        spawn_robot,
        spawn_police,
        bridge,
        navigation,
        traffic_police,
        rviz_node,
    ])
