
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():



    ld = LaunchDescription()



    simple_commander_node = Node(
        package = "simple_igoneo_commander",
        executable="simple_commander"
    )
    
    log_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','-o','IgoNeo_Lat_response_S20000T' ,'/odom', '/joint_command', '/joint_states'],
        output='screen'
    )


    ld.add_action(simple_commander_node)
    #ld.add_action(log_process)
    return ld