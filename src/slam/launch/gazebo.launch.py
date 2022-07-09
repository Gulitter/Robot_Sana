import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
#urdf를 실행시키기 위해서는 ExecuteProcess를 import해줘야한다.

def generate_launch_description():
# Constants for paths to different files and folders
	gazebo_models_path = 'models'
	package_name = 'slam'
	robot_file = "rlautocar.urdf"
	robot_name_in_model = 'rlautocar'
	rviz_config_file_path = 'rviz/slam.rviz'
	urdf_file_path = 'urdf/rlautocar.urdf'
	world_file_path = 'worlds/empty.world'
	# Pose where we want to spawn the robot
	spawn_x_val = '0.0'
	spawn_y_val = '0.0'
	spawn_z_val = '0.1'
	spawn_yaw_val = '0.00'
	############ You do not need to change anything below this line #############
	# Set the path to different files and folders. 
	pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
	pkg_share = FindPackageShare(package=package_name).find(package_name)
	default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
	default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
	world_path = os.path.join(pkg_share, world_file_path)
	gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
	os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
	default_model_path = os.path.join(get_package_share_directory(package_name), "urdf", robot_file)
	# Launch configuration variables specific to simulation
	gui = LaunchConfiguration('gui')
	headless = LaunchConfiguration('headless')
	namespace = LaunchConfiguration('namespace')
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	urdf_model = LaunchConfiguration('urdf_model')
	use_namespace = LaunchConfiguration('use_namespace')
	use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
	use_rviz = LaunchConfiguration('use_rviz')
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	use_simulator = LaunchConfiguration('use_simulator')
	world = LaunchConfiguration('world')
	robot_description = open(default_model_path, 'r').read()
	# Declare the launch arguments 
	declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
	name='gui',
	default_value='True',
	description='Flag to enable joint_state_publisher_gui')
	declare_namespace_cmd = DeclareLaunchArgument(
	name='namespace',
	default_value='',
	description='Top-level namespace')
	declare_use_namespace_cmd = DeclareLaunchArgument(
	name='use_namespace',
	default_value='false',
	description='Whether to apply a namespace to the navigation stack')
	declare_rviz_config_file_cmd = DeclareLaunchArgument(
	name='rviz_config_file',
	default_value=default_rviz_config_path,
	description='Full path to the RVIZ config file to use')
	declare_simulator_cmd = DeclareLaunchArgument(
	name='headless',
	default_value='False',
	description='Whether to execute gzclient')
	declare_urdf_model_path_cmd = DeclareLaunchArgument(
	name='urdf_model', 
	default_value=default_urdf_model_path, 
	description='Absolute path to robot urdf file')
	declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
	name='use_robot_state_pub',
	default_value='True',
	description='Whether to start the robot state publisher')
	declare_use_rviz_cmd = DeclareLaunchArgument(
	name='use_rviz',
	default_value='True',
	description='Whether to start RVIZ')
	declare_use_sim_time_cmd = DeclareLaunchArgument(
	name='use_sim_time',
	default_value='true',
	description='Use simulation (Gazebo) clock if true')
	declare_use_simulator_cmd = DeclareLaunchArgument(
	name='use_simulator',
	default_value='True',
	description='Whether to start the simulator')
	declare_world_cmd = DeclareLaunchArgument(
	name='world',
	default_value=world_path,
	description='Full path to the world model file to load')
	# Subscribe to the joint states of the robot, and publish the 3D pose of each link. 
	start_robot_state_publisher_cmd = Node(
	package='robot_state_publisher',
	executable='robot_state_publisher',
	parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
	arguments=[default_model_path])
	# Publish the joint states of the robot
	start_joint_state_publisher_cmd = Node(
	package='joint_state_publisher',
	executable='joint_state_publisher',
	name='joint_state_publisher',
	condition=UnlessCondition(gui))
	# Launch RViz
	start_rviz_cmd = Node(
	package='rviz2',
	executable='rviz2',
	name='rviz2',
	output='screen',
	arguments=['-d', rviz_config_file])
	joint_state_publisher_gui_node = Node(
	package='joint_state_publisher_gui',
	executable='joint_state_publisher_gui',
	condition=IfCondition(LaunchConfiguration('gui')))
	# Start Gazebo server
	start_gazebo_server_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
	condition=IfCondition(use_simulator),
	launch_arguments={'world': world}.items())
	# Start Gazebo client 
	start_gazebo_client_cmd = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
	condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
	#ros2 run gazebo_ros spawn_entity.py -entity rlautocar -topic robot_description -x 0.0 -y 0.0 -z 0.0 -Y 0.0


	return LaunchDescription([
	declare_use_joint_state_publisher_cmd,
	declare_namespace_cmd,
	declare_use_namespace_cmd,
	declare_simulator_cmd,
	declare_urdf_model_path_cmd,
	declare_use_robot_state_pub_cmd,
	declare_use_sim_time_cmd,
	declare_use_simulator_cmd,
	declare_world_cmd,
	start_robot_state_publisher_cmd,
	#joint_state_publisher_gui_node,
	start_joint_state_publisher_cmd,
	IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')), condition=IfCondition(use_simulator), launch_arguments={'world': world}.items()),
	IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')), condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))),
	ExecuteProcess(cmd=["ros2", "run", "gazebo_ros", "spawn_entity.py",'-entity', robot_name_in_model, '-topic', 'robot_description', '-x', spawn_x_val, '-y', spawn_y_val, '-z', spawn_z_val, '-Y', spawn_yaw_val], output="screen")
	])
