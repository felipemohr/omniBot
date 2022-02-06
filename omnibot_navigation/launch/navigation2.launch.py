import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():

  omnibot_navigation_pkg_share = FindPackageShare('omnibot_navigation').find('omnibot_navigation')
  nav2_bt_navigator_pkg_share = FindPackageShare('nav2_bt_navigator').find('nav2_bt_navigator')

  namespace = LaunchConfiguration('namespace')
  use_sim_time = LaunchConfiguration('use_sim_time')
  autostart = LaunchConfiguration('autostart')
  params_file = LaunchConfiguration('params_file')
  default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
  map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

  lifecycle_nodes = ['controller_server',
                     'planner_server',
                     'recoveries_server',
                     'bt_navigator',
                     'waypoint_follower']

  remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

  # Create our own temporary YAML files that include substitutions
  param_substitutions = {
      'use_sim_time': use_sim_time,
      'default_bt_xml_filename': default_bt_xml_filename,
      'autostart': autostart,
      'map_subscribe_transient_local': map_subscribe_transient_local}

  configured_params = RewrittenYaml(
          source_file=params_file,
          root_key=namespace,
          param_rewrites=param_substitutions,
          convert_types=True)

  nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
  )

  nav2_planner = Node(
      package='nav2_planner',
      executable='planner_server',
      name='planner_server',
      output='screen',
      parameters=[configured_params],
      remappings=remappings
  )

  nav2_recoveries = Node(
      package='nav2_recoveries',
      executable='recoveries_server',
      name='recoveries_server',
      output='screen',
      parameters=[configured_params],
      remappings=remappings
  )

  nav2_bt_navigator = Node(
      package='nav2_bt_navigator',
      executable='bt_navigator',
      name='bt_navigator',
      output='screen',
      parameters=[configured_params],
      remappings=remappings
  )

  nav2_waypoint_follower = Node(
      package='nav2_waypoint_follower',
      executable='waypoint_follower',
      name='waypoint_follower',
      output='screen',
      parameters=[configured_params],
      remappings=remappings
  )

  nav2_lifecycle_manager = Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_navigation',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time},
                  {'autostart': autostart},
                  {'node_names': lifecycle_nodes}]
  )


  return launch.LaunchDescription([
    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    DeclareLaunchArgument(
            name='namespace', default_value='',
            description='Top-level namespace'
    ),
    DeclareLaunchArgument(
        name='use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ),
    DeclareLaunchArgument(
        name='autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    ),
    DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(omnibot_navigation_pkg_share, 'config', 'nav2.yaml'),
        description='Full path to the ROS2 parameters file to use'
    ),
    DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=os.path.join(
            nav2_bt_navigator_pkg_share,
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use'
    ),
    DeclareLaunchArgument(
        name='map_subscribe_transient_local', default_value='false',
        description='Whether to set the map subscriber QoS to transient local'
    ),
    nav2_controller,
    nav2_planner,
    nav2_recoveries,
    nav2_bt_navigator,
    nav2_waypoint_follower,
    nav2_lifecycle_manager
  ])
