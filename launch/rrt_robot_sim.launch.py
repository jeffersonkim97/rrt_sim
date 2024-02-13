from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import os
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # gz -> ros2
    # Receive robot pose data
    # ros2 run ros_gz_bridge parameter_bridge /model/target/pose@geometry_msgs/msg/Pose@gz.msgs.Pose
    gz_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/model/target/pose@geometry_msgs/msg/Pose@gz.msgs.Pose'],
        output='screen'
    )

    # ros2 -> gz
    # Export Twist data
    # ros2 run ros_gz_bridge parameter_bridge /target/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
    gz_twist_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # Export Camera Image
    image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/ptzCam1/camera_x1@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/ptzCam2/camera_x1@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # Export Camera Image
    camera_pancmd_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/cam1/pan_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                   ],
        output='screen'
    )

    ld.add_action(gz_pose_bridge)
    ld.add_action(gz_twist_bridge)
    ld.add_action(image_bridge)
    ld.add_action(camera_pancmd_bridge)

    return ld
