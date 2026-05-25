import os
import math
import random
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    pkg_diffbot = get_package_share_directory('diffbot_sim')
    num_robots = 10
    positions = []

    # Grid parameters for 10 robots (5 columns x 2 rows)
    cols = 5
    rows = 2
    spacing = 1.5 
    jitter = 0.3  

    for i in range(num_robots):
        col = i % cols
        row = i // cols
        
        # Center the grid at (0,0)
        x_base = (col - (cols - 1) / 2) * spacing
        y_base = (row - (rows - 1) / 2) * spacing
        
        # Add jitter to avoid perfect lines
        x = x_base + random.uniform(-jitter, jitter)
        y = y_base + random.uniform(-jitter, jitter)
        
        positions.append([x, y])

    xacro_file = os.path.join(pkg_diffbot, 'urdf', 'obs1.urdf.xacro')
    nodes = []

    for i in range(num_robots):
        robot_name = f'obs{i+1}'
        
        doc = xacro.process_file(xacro_file, mappings={'robot_name': robot_name})
        robot_description_config = doc.toprettyxml(indent='  ')

        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}],
            output='screen'
        )

        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-topic', f'/{robot_name}/robot_description',
                '-x', str(positions[i][0]),
                '-y', str(positions[i][1]),
                '-z', '0.05'
            ],
            output='screen'
        )

        delayed_spawn = TimerAction(
            period=float(i * 0.3), 
            actions=[spawn_node]
        )

        nodes.append(rsp_node)
        nodes.append(delayed_spawn)

    movement_node = Node(
        package='diffbot_sim',
        executable='dynamic_obstacles',  
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    nodes.append(movement_node)

    return LaunchDescription(nodes)