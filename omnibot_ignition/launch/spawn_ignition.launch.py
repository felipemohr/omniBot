import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

  omnibot_description_pkg_share = FindPackageShare('omnibot_description').find('omnibot_description')

  default_model_path = os.path.join(omnibot_description_pkg_share, 'urdf/omnibot.urdf.xacro')
  default_rviz_config_path = os.path.join(omnibot_description_pkg_share, 'rviz/config.rviz')

  robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
              os.path.join(omnibot_description_pkg_share, 'launch', 'start_robot.launch.py')),
            launch_arguments={'model': LaunchConfiguration('model'),
                              'use_rviz': LaunchConfiguration('use_rviz'),
                              'rviz_config': LaunchConfiguration('rviz_config'),
                              'use_sim_time': 'true'}.items(),
  )

  spawn_robot = Node(
    package='ros_ign_gazebo',
    executable='create',
    arguments=['-name', 'omnibot',
               '-topic', 'robot_description',
               '-z', '0.25',  # Argumento para posição
              ],
    output='screen'
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
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/model/omnibot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/omnibot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            ],
    remappings=[
        ('/world/omni_world/model/omnibot/joint_state', 'joint_states'),
        ('/model/omnibot/cmd_vel', 'cmd_vel'),
        ('/model/omnibot/odometry', 'odom')
    ],
    output='screen'
  )

  tf2_lidar_transform = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "omnibot/base_link/gpu_lidar"]
  )

  tf2_camera_transform = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "camera_link", "omnibot/base_link/camera"]
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                         description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='use_rviz', default_value='true',
                                         description='Use RViz if true'),
    launch.actions.DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                         description='Absolute path to rviz config file'),
    robot,
    spawn_robot,
    ign_ros_bridge,
    tf2_lidar_transform,
    tf2_camera_transform
  ])
