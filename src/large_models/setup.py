import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'large_models'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vocal_detect = large_models.vocal_detect:main',
            'agent_process = large_models.agent_process:main',
            'tts_node = large_models.tts_node:main',

            'vllm_smart_housekeeper = large_models.vllm_smart_housekeeper:main',           
            'vllm_with_camera = large_models.vllm_with_camera:main',
            'vllm_obstacle_avoidance = large_models.vllm_obstacle_avoidance:main',
            'llm_control_move = large_models.llm_control_move:main',
            'llm_visual_patrol = large_models.llm_visual_patrol:main',
            'llm_color_track = large_models.llm_color_track:main',
        ],
    },
)
