from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Prompt the user for x_pose and y_pose values
    x_pose = (input('Enter the value for x_pose: '))
    y_pose = (input('Enter the value for y_pose: '))
    yaw = 0.0

     # Pronmpt task choosing:
    task = (input('Choose your task: \n0: MAPPING \n1: SANITIZATION\n'))



     # Declare the launch arguments
    global_amcl_flag = '0'

    if task == '0':
        declare_slam_cmd = DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Enable SLAM'
        )
    else:
        declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Enable SLAM')

        global_amcl_flag = input('Do you want to use global amcl localization? \n0: NO\n1: YES\n')

    with open('task_parameters.txt', 'w') as file:
        file.write(f'x_pose = {x_pose}\n')  # La variabile e il valore sono separati da "="
        file.write(f'y_pose = {y_pose}\n')  # Anche qui, la variabile e il valore sono separati da "="
        file.write(f'yaw = {yaw}\n')
        file.write(f'task = {task}\n')
        file.write(f'global_amcl_flag = {global_amcl_flag}\n')
          
        

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/ARS_Project_map.yaml'),
        description='MAP'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run in headless mode'
    )
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

 
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw angle for the robot (in radians)')

    declare_amcl_node_cmd = Node(
            package='nav2_amcl',
            namespace='',
            executable='amcl',
            name='localization')
    
    

    
    
    # Export the TURTLEBOT3_MODEL environment variable
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # Append to the GAZEBO_MODEL_PATH environment variable
    gazebo_model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    os.environ['GAZEBO_MODEL_PATH'] += f":{gazebo_model_path}"

    # Get the path to the tb3_simulation_launch.py file
    tb3_simulation_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'tb3_simulation_launch.py'
    )

    # Include the tb3_simulation launch file with the declared arguments
    if task == '0':
        tb3_simulation_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_simulation_launch_file),
            launch_arguments={
                'slam': LaunchConfiguration('slam'),
                'headless': LaunchConfiguration('headless'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose')
            }.items()
        )

    else:
        tb3_simulation_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_simulation_launch_file),
            launch_arguments={
                'slam': LaunchConfiguration('slam'),
                'map': LaunchConfiguration('map'),
                'headless': LaunchConfiguration('headless'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose')
            }.items()
        )

    if task == '0':
        return LaunchDescription([
            declare_slam_cmd,
            declare_headless_cmd,
            declare_x_pose_cmd,
            declare_y_pose_cmd,
            declare_yaw_cmd,
            tb3_simulation_include,
        ])
    
    elif task == '1':
        return LaunchDescription([
            declare_slam_cmd,
            declare_headless_cmd,
            declare_x_pose_cmd,
            declare_y_pose_cmd,
            declare_yaw_cmd,
            declare_map_cmd,
            declare_amcl_node_cmd,

            
            tb3_simulation_include,

            

        ])
    
    else:
        print("!!!!!!!!!!! ERROR: UNVALID CHOICE !!!!!!!!!!!!!!!!!")
        return -1
        
