import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():

  ros_ign_gazebo_pkg_share = FindPackageShare('ros_ign_gazebo').find('ros_ign_gazebo')
  omnibot_description_pkg_share = FindPackageShare(package='omnibot_description').find('omnibot_description')

  default_world_path = os.path.join(omnibot_description_pkg_share, 'worlds/omni_world.sdf')
  default_model_urdf_path = os.path.join(omnibot_description_pkg_share, 'urdf/omnibot.urdf.xacro')
  default_rviz_config_path = os.path.join(omnibot_description_pkg_share, 'rviz/config.rviz')


  ignition = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
               os.path.join(ros_ign_gazebo_pkg_share, 'launch', 'ign_gazebo.launch.py')),
             launch_arguments={'ign_args': ['-r ', LaunchConfiguration('world')]}.items(),
  )

  spawn_entity = Node(
    package='ros_ign_gazebo',
    executable='create',
    arguments=['-name', 'omnibot',
               '-topic', 'robot_description',
               '-z', '0.25',
              ],
    output='screen'
  )

  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher'
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', default_model_urdf_path]),
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
  )
  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')]
  )

  ign_ros_bridge = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/omni_world/model/omnibot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
            ],
    remappings=[
        ('/world/omni_world/model/omnibot/joint_state', 'joint_states'),
    ],
    output='screen'
  )

  tf2_lidar_transform = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "omnibot/base_link/gpu_lidar"]
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='world', default_value=default_world_path,
                                         description='Absolute path to world sdf file'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                         description='Absolute path to rviz config file'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time',default_value='true',
                                         description='Use simulation (Gazebo) clock if true'),
    ignition,
    spawn_entity,
    ign_ros_bridge,
    joint_state_publisher,
    robot_state_publisher,
    tf2_lidar_transform,
    TimerAction(period=2.0, actions=[rviz_node]),
  ])