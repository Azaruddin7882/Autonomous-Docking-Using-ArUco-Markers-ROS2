import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('docking_robot')
    urdf_file = package_share_directory + '/urdf/robot.urdf'

    return LaunchDescription([
        Node(
            package='docking_robot',
            executable='aruco_marker_detection',
            name='aruco_marker_detection',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='docking_robot',
            executable='docking_logic',
            name='docking_logic',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[urdf_file]
        ),
    ])
