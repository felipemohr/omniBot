import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

  omnibot_navigation_pkg_share = FindPackageShare('omnibot_navigation').find('omnibot_navigation')

  async_slam_toolbox = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    parameters=[
      LaunchConfiguration('slam_params_file'),
      {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ],
    output='screen')

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='use_sim_time',default_value='true',
                                         description='Use simulation (Gazebo) clock if true'),
    launch.actions.DeclareLaunchArgument(name='slam_params_file',
                                         default_value=os.path.join(omnibot_navigation_pkg_share,
                                                                    'config/slam_online_async.yaml'),
                                         description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
    async_slam_toolbox
  ])
