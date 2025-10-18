import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包路径
    app_package_path = get_package_share_directory('app')
    peripherals_package_path = get_package_share_directory('peripherals')
    controller_package_path = get_package_share_directory('controller')

    # 定义各个节点的 launch 文件
    gesture_control_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/gesture_control_node.launch.py')
        )
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

    line_following_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/line_following_node.launch.py')
        )
    )

    object_tracking_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/object_tracking_node.launch.py')
        )
    )
    
    avoidance_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/avoidance_node.launch.py')
        )
    )

    # ar_app_node_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(app_package_path, 'launch/ar_app_node.launch.py')
    #     )
    # )

    qrcode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/qrcode.launch.py')
        )
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        gesture_control_node_launch,
        #usb_cam_launch,
        object_tracking_node_launch,
        line_following_node_launch,
        avoidance_node_launch,
        #controller_launch,
        qrcode_launch
    ])

if __name__ == '__main__':
    # 创建一个 LaunchDescription 对象
    ld = generate_launch_description()

    # 创建一个 LaunchService 对象
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
