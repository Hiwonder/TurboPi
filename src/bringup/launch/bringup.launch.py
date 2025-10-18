import os
import time
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    # 获取包的路径
    controller_package_path = get_package_share_directory('controller')
    app_package_path = get_package_share_directory('app')
    peripherals_package_path = get_package_share_directory('peripherals')

    # 启动 controller 的 launch 文件
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )
    
    # 启动 web_video_server 进程
    web_video_server_launch = ExecuteProcess(
        cmd=['ros2', 'run', 'web_video_server', 'web_video_server'],
        output='screen'
    )

    # 启动 rosbridge_websocket
    rosbridge_websocket_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    # 启动摄像头相关节点
    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/usb_cam.launch.py')),
    )


    sonar_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/sonar_controller_node.launch.py')),
    )

    # 启动 start_app 的 launch 文件
    start_app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/start_app.launch.py')),
    )

    startup_check_node = Node(
        package='bringup',
        executable='startup_check',
        output='screen',
    )

    delayed_startup_check = TimerAction(
        period=10.0,  # 10秒后执行 startup_check_node
        actions=[startup_check_node]
    )

    return [
        web_video_server_launch,
        rosbridge_websocket_launch,
        controller_launch,
        usb_cam_launch,
        #sonar_controller_launch,
        start_app_launch,
        delayed_startup_check  
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    # 创建LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
