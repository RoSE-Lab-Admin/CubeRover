from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import re
import tempfile

import yaml


def select_planner_in_bt(bt_xml_path, nav2_config_path):
    # Single source of truth for which planner navigation uses: the first entry
    # of planner_server's planner_plugins in the nav2 yaml. Writes a copy of the
    # BT with every ComputePathToPose planner_id set to it, returns its path.
    with open(nav2_config_path) as f:
        config = yaml.safe_load(f)
    planner_id = config['planner_server']['ros__parameters']['planner_plugins'][0]
    with open(bt_xml_path) as f:
        xml = f.read()
    xml = re.sub(r'(<ComputePathToPose\b[^>]*\bplanner_id=")[^"]*(")',
                 rf'\g<1>{planner_id}\g<2>', xml)
    out_dir = tempfile.mkdtemp(prefix='nav2_stack_bt_')
    out_path = os.path.join(out_dir, os.path.basename(bt_xml_path))
    with open(out_path, 'w') as f:
        f.write(xml)
    print(f'[nav2.launch.py] ComputePathToPose planner_id -> {planner_id} ({out_path})')
    return out_path


def launch_setup(context):
    robot_frame = LaunchConfiguration('robot_frame').perform(context)

    bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_stack'),
        'behavior_trees',
        'navigate_recovery.xml'
    ])

    nav2_config = PathJoinSubstitution([
        FindPackageShare('nav2_stack'),
        'config',
        'nav2_param2.yaml'
    ])

    bt_xml = select_planner_in_bt(bt_xml.perform(context), nav2_config.perform(context))

    map_config = PathJoinSubstitution([
        FindPackageShare('nav2_stack'),
        'maps',
        'map.yaml'
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_config,
            'use_sim_time': False
        }]
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'planner_server',
                           'behavior_server', 'bt_navigator',
                           'waypoint_follower']
        }]
    )

    # controller_server has its own manager so a new dynamics model can be
    # loaded by cycling just it (autonomous_trials.reload_controller ->
    # RESET/STARTUP on this manager) instead of the whole stack, whose
    # planner_server alone takes ~27 s to reconfigure.
    lifecycle_mgr_controller = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['controller_server']
        }]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config, {
            'use_sim_time': False,
            'robot_base_frame': robot_frame,
        }]
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config, {
            'use_sim_time': False,
            'robot_base_frame': robot_frame,
        }]
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config, {
            'use_sim_time': False,
            'robot_base_frame': robot_frame,
        }]
    )

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config, {
            'use_sim_time': False,
            'robot_base_frame': robot_frame,
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_xml
        }]
    )

    waypoint = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config, {
            'use_sim_time': False,
        }]
    )

    # Static map->odom transform (identity). Assumes robot starts at map origin.
    # Replace with AMCL or SLAM if relocalization is needed.
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    return [
        map_to_odom_tf,
        world_to_map_tf,
        map_server,
        planner,
        controller,
        behavior,
        bt_nav,
        waypoint,
        lifecycle_mgr,
        lifecycle_mgr_controller,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_frame', default_value='FitRosey_V1'),
        OpaqueFunction(function=launch_setup),
    ])
