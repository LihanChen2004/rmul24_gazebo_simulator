import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

	start_rviz2 = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', os.path.join(get_package_share_directory('rmul24_gazebo_simulator'), 'rviz', 'visualize.rviz')]
	)

	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(start_rviz2)

	return ld