from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

path_to_src = get_package_share_directory('relaxed_ik_ros2')
default_setting_file_path = path_to_src + '/relaxed_ik_core/configs/settings.yaml'
ur5_setting_file_path = path_to_src + '/relaxed_ik_core/configs/example_settings/ur5.yaml'

setting_file_path = ur5_setting_file_path

def load_setting_file_path(context: LaunchContext, setting_file_path_launch_config):
    # Convert to string
    setting_file_path_str = context.perform_substitution(setting_file_path_launch_config)
    print("setting_file_path_str: ", setting_file_path_str)
    setting_file_path.write_text(setting_file_path_str)

def generate_launch_description():

    # currently not working
    setting_file_path_arg = DeclareLaunchArgument(
        'setting_file_path',
        default_value = default_setting_file_path,
        description = 'Path to the setting file'
    )

    OpaqueFunction(function=load_setting_file_path, args=[LaunchConfiguration('setting_file_path')])

    # Load the infomation
    setting_file = open(setting_file_path, 'r')
    settings = yaml.load(setting_file, Loader=yaml.FullLoader)

    urdf_path = path_to_src + '/relaxed_ik_core/configs/urdfs/' + settings["urdf"]
    urdf_file = open(urdf_path, 'r')
    urdf_string = urdf_file.read()

    print(str(setting_file_path))

    return LaunchDescription([
        setting_file_path_arg, 
        Node(
            package='relaxed_ik_ros2',
            namespace='',
            executable='relaxed_ik_rust.py',
            name='relaxed_ik_rust',
            parameters=[
                {'use_visualization': 'True'}
            ]
        ), 
        Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2',
            # output='screen',
            arguments=['-d', path_to_src + "/rviz/relaxed_ik_viewer.rviz"],
        ),
        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path],
            parameters=[{'publish_frequency': 50.0, 
                        'robot_description': urdf_string}]
        ),
        Node(
            package='relaxed_ik_ros2',
            namespace='',
            executable='rviz_viewer.py',
            name='rviz_viewer',
            output='screen',
            parameters=[{'setting_file_path': setting_file_path}]
        ),
    ])