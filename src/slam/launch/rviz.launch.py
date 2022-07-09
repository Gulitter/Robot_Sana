import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file = "slam.rviz"
    robot_file = "slam.urdf"
    package_name = "slam"


    default_model_path = os.path.join(get_package_share_directory(package_name), "urdf", robot_file)

    default_rviz_config_path = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file)
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', description='Flag to enable joint_state_publisher_gui')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path), description='Absolute path to robot urdf file')

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), description='Absolute path to rviz config file')

    robot_description = open(default_model_path, 'r').read()


    robot_state_publisher_node = Node(

    package='robot_state_publisher',

    executable='robot_state_publisher',

    parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],

    arguments=[default_model_path],

    )
# Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui

    joint_state_publisher_node = Node(

    package='joint_state_publisher',

    executable='joint_state_publisher',

    condition=IfCondition(LaunchConfiguration('gui'))

    )

    joint_state_publisher_gui_node = Node(

    package='joint_state_publisher_gui',

    executable='joint_state_publisher_gui',

    condition=IfCondition(LaunchConfiguration('gui'))

    )


    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,

        robot_state_publisher_node,

        ExecuteProcess(cmd=["ros2", "run", "rviz2", "rviz2", "-d", default_rviz_config_path], output="screen")

])
