from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    frame_id = LaunchConfiguration('frame_id')
    publish_tf = LaunchConfiguration('publish_tf')
    carto_dir = LaunchConfiguration('cartographer_config_dir')
    carto_file = LaunchConfiguration('cartographer_config_basename')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyAMA0:115200',
                              description='MAVROS FCU URL'),
        DeclareLaunchArgument('frame_id', default_value='sf45b_lidar',
                              description='Frame id for LaserScan'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Publish static base_link TF'),
        DeclareLaunchArgument('cartographer_config_dir', default_value='/workspace/ros2_ws/src/slam_bridge/config',
                              description='Cartographer configuration directory'),
        DeclareLaunchArgument('cartographer_config_basename', default_value='sf45b_2d.lua',
                              description='Cartographer lua config file'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),

        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{'fcu_url': fcu_url, 'gcs_url': '', 'target_system_id': 1, 'target_component_id': 1, 'system_id': 1, 'component_id': 1, 'use_sim_time': use_sim_time}],
            arguments=['--ros-args'],
        ),

        Node(
            package='slam_bridge',
            executable='slam_bridge',
            name='slam_bridge',
            output='screen',
            parameters=[{'frame_id': frame_id, 'use_sim_time': use_sim_time}],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', carto_dir,
                       '-configuration_basename', carto_file,
                       '--ros-args', '-r', 'scan:=/scan']
        ),

        # Cartographer occupancy grid publisher (optional map generation)
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05}],
        ),

        # Optional static transform (map->base_link) placeholder (only if requested)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_link',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            condition=IfCondition(publish_tf)
        ),
    ])
