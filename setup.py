
from setuptools import find_packages, setup

package_name = 'docking_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['package.xml']),  # Path to package.xml
        ('share/' + package_name + '/launch',
         ['src/' + package_name + '/launch/docking_launch.py',
          'src/' + package_name + '/launch/gazebo_launch.py']),
        ('share/' + package_name + '/urdf',
         ['src/' + package_name + '/urdf/robot.urdf']),
        ('share/' + package_name + '/worlds',
         ['src/' + package_name + '/worlds/docking_world.world']),
        ('share/' + package_name + '/rviz',
         ['src/' + package_name + '/rviz/rviz_config.rviz']),
        ('share/' + package_name + '/resource/markers',
         ['src/' + package_name + '/resource/markers/aruco_marker.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azar',
    maintainer_email='azaruddin7882@gmail.com',
    description='ROS2 package for autonomous docking using ArUco markers',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_marker_detection = docking_robot.docking_node.aruco_marker_detection:main',
            'docking_logic = docking_robot.docking_node.docking_logic:main',
        ],
    },
)










