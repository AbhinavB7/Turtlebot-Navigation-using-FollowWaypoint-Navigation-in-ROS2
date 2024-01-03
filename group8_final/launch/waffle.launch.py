from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():
   
    parameter_file = os.path.join(
    get_package_share_directory('group8_final'),
    'config',
    'waypoint_params.yaml'
    )
    
    ld = LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
    ])
    
    Turtlebot_navigation = Node(
        package="group8_final",
        executable="Turtlebot_navigation",
        parameters=[parameter_file, {'use_sim_time': True}],
        output='screen',
    )
  
    ld.add_action(Turtlebot_navigation)
    return ld