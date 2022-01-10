import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

  ros_ign_gazebo_pkg_share = FindPackageShare('ros_ign_gazebo').find('ros_ign_gazebo')
  omnibot_ignition_pkg_share = FindPackageShare('omnibot_ignition').find('omnibot_ignition')
  omnibot_description_pkg_share = FindPackageShare('omnibot_description').find('omnibot_description')

  default_world_path = os.path.join(omnibot_ignition_pkg_share, 'worlds/omni_world.sdf')
  default_model_path = os.path.join(omnibot_description_pkg_share, 'urdf/omnibot.urdf.xacro')
  default_rviz_config_path = os.path.join(omnibot_description_pkg_share, 'rviz/config.rviz')

  ignition = IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                os.path.join(ros_ign_gazebo_pkg_share, 'launch', 'ign_gazebo.launch.py')),
              launch_arguments={'ign_args': ['-r ', LaunchConfiguration('world')]}.items(),
  )

  spawn_ignition = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                      os.path.join(omnibot_ignition_pkg_share, 'launch', 'spawn_ignition.launch.py')),
                    launch_arguments={'model': LaunchConfiguration('model'),
                                      'use_rviz': LaunchConfiguration('use_rviz'),
                                      'rviz_config': LaunchConfiguration('rviz_config')}.items(),
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='world', default_value=default_world_path,
                                         description='Absolute path to world sdf file'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                         description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='use_rviz', default_value='false',
                                         description='Use RViz if true'),
    launch.actions.DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                         description='Absolute path to rviz config file'),
    ignition,
    spawn_ignition
  ])
