import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import ExecuteProcess


def generate_launch_description():
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            '/home/ros_workspace/src/mpcc_planner/mpcc_local_planner','maps', "ware_house.yaml"
        ),
    )

    ld = LaunchDescription()

    mpcc_controller_node = Node(
        package="mpcc_local_planner",
        executable="mpcc"
    )

    local_path_planner_node = Node(
        package="local_path_planner",
        executable="path_planner"
    )

    cost_map_server_node = Node(
        package = "cost_map_server",
        executable="cost_map_server"
    )

    static_collision_node = Node(
        package = "static_collision_avoidance",
        executable="static_collision_avoidance"
    )
    
    dynamic_collision_node = Node(
        package = "dynamic_collision_avoidance",
        executable="dynamic_collision_avoidance"
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_dir}])
    
    log_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','-o','Map1_Test3_obst_fast' ,'/vehicle_odom', '/joint_command', '/joint_states', '/obs_predicted_traj',
             '/mpcc_time', '/mpcc_predicted_traj', '/collision_pts', '/mpcc_feedback' ,'/mpcc_torque', "/path_boundary", "/collision_time", "/step_time"],
        output='screen'
    )

    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True
    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    ld.add_action(map_server_cmd)
    ld.add_action(cost_map_server_node)
    #ld.add_action(mpcc_controller_node)
    #ld.add_action(local_path_planner_node)
    ld.add_action(dynamic_collision_node)
    ld.add_action(start_lifecycle_manager_cmd)
    #ld.add_action(log_process)

    return ld