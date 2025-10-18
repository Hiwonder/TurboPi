from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)

    # ros_robot_controller node
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',
        parameters=[{'imu_frame': imu_frame}]
    )

    # mecanum_chassis node
    mecanum_chassis_node = Node(
        package='controller',
        executable='mecanum',
        name='mecanum_chassis_node',
        output='screen',
        parameters=[],
        remappings=[]
    )

    return LaunchDescription([
        imu_frame_arg,
        ros_robot_controller_node,
        mecanum_chassis_node
    ])
