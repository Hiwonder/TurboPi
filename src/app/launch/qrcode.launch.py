import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    peripherals_package_path = get_package_share_directory('peripherals')
    controller_package_path = get_package_share_directory('controller')

    # 定义 qrcode 节点
    qrcode_node = Node(
        package='app',
        executable='qrcode',
        output='screen',
    )

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/usb_cam.launch.py')
        )
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')
        )
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        #usb_cam_launch,
        qrcode_node,
        #controller_launch,
    ])

if __name__ == '__main__':
    # 创建一个 LaunchDescription 对象
    ld = generate_launch_description()

    # 创建一个 LaunchService 对象
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
