import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_swarm_description = get_package_share_directory('swarm_description')
    
    world_file = os.path.join(pkg_swarm_description, 'worlds', 'swarm_world.sdf')
    urdf_file = os.path.join(pkg_swarm_description, 'urdf', 'turtlebot3_swarm.urdf.xacro')

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # Spawn 50 robots
    nodes = [gazebo]
    num_robots = 50

    for i in range(num_robots):
        robot_name = f'robot_{i}'
        x_pos = (i % 10) * 1.0  # 10 robots per row
        y_pos = (i // 10) * 1.0

        # Robot State Publisher
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': os.popen(f'xacro {urdf_file} robot_name:={robot_name}').read()}]
        ))

        # Spawn entity in Gazebo
        nodes.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-topic', f'/{robot_name}/robot_description',
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', '0.1'
            ],
            output='screen'
        ))

    # Stigmergy Server Node
    stigmergy_server = Node(
        package='stigmergy_server',
        executable='environment_node',
        name='stigmergy_server',
        output='screen'
    )
    nodes.append(stigmergy_server)

    return LaunchDescription(nodes)
