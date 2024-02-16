from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    

    with open('task_parameters.txt', 'r') as file:
        file_content = file.read()

    # Dividi il contenuto del file in righe
    file_rows = file_content.split('\n')

    # Inizializza un dizionario vuoto
    parameters = {}

    # Loop attraverso le righe
    for j in range(len(file_rows)-1):
        variable, value = file_rows[j].split(' = ')
        
        variable = variable.strip()
        
        parameters[variable] = value


    task = parameters['task']
    x_pose = parameters['x_pose']
    y_pose = parameters['y_pose']
    yaw = parameters['yaw']
    global_amcl_flag = parameters['global_amcl_flag']


    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value=str(x_pose),
        description='Initial x position for the robot'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value=str(y_pose),
        description='Initial y position for the robot'
    )

    declare_global_amcl_flag_cmd = DeclareLaunchArgument(
        'global_amcl_flag',
        default_value=str(global_amcl_flag),
        description='Flag for initialize amcl'
    )
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw angle for the robot (in radians)')

    declare_mapping_node_cmd = Node(
            package='nav2_wfd',
            namespace='',
            executable='explore',
            name='mapping')
    
    
    declare_power_pub_node_cmd = Node(
            package='tasks',
            namespace='',
            executable='power_publisher',
            name='power_pub')
    
    declare_energy_pub_node_cmd = Node(
            package='tasks',
            namespace='',
            executable='energy_publisher',
            name='energy_pub')
    
    declare_energy_nav_node_cmd = Node(
            package='tasks',
            namespace='',
            executable='energy_navigation',
            name='energy_nav')
    
    declare_init_localization_node_cmd = Node(
            package='tasks',
            namespace='',
            executable='init_localization',
            parameters=[
                {'x_pose': LaunchConfiguration('x_pose')},
                {'y_pose': LaunchConfiguration('y_pose')},
                {'global_amcl_flag': LaunchConfiguration('global_amcl_flag')}
            ],
            name='localization_node')


    if task == '0':
        return LaunchDescription([
            declare_x_pose_cmd,
            declare_y_pose_cmd,
            declare_yaw_cmd,
            declare_mapping_node_cmd
        ])
    
    elif task == '1':
        return LaunchDescription([
            declare_x_pose_cmd,
            declare_y_pose_cmd,
            declare_yaw_cmd,
            declare_global_amcl_flag_cmd,
 

            declare_init_localization_node_cmd,
            
            
            declare_power_pub_node_cmd,
            declare_energy_pub_node_cmd,
            
            declare_energy_nav_node_cmd
        ])
    
    else:
        print("!!!!!!!!!!! ERROR: UNVALID CHOICE !!!!!!!!!!!!!!!!!")
        return -1
        
