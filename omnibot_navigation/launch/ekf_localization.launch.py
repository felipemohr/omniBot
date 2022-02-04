import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

  omnibot_navigation_pkg_share = FindPackageShare('omnibot_navigation').find('omnibot_navigation')

  robot_localization = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
      os.path.join(omnibot_navigation_pkg_share, 'config/ekf.yaml'),
      {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ]
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='use_sim_time',default_value='true',
                                         description='Use simulation (Gazebo) clock if true'),
    robot_localization
  ])
