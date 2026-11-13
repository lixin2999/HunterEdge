import os

from setuptools import find_packages, setup

package_name = 'sil_bridge_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sil_bridge_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/sil_bridge.launch.py']),
        ('share/' + package_name + '/scripts', ['scripts/sil-bridge.service']),
        # 可执行入口装到 lib/<pkg>/：**不依赖 setuptools 发行版元数据**
        # （console_scripts 入口包装在 --symlink-install 下会抛 PackageNotFoundError，
        #   HUNTER-001 实机上 remote-agent/ota-agent 因此陷入 activating 重启循环）
        (os.path.join('lib', package_name), ['scripts/sil_bridge_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HUNTER Development Team',
    maintainer_email='developer@hunter.ai',
    description='SIL bridge agent - CARLA simulation sensors to HunterEdge ROS topics and /cmd_vel back over Kafka',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sil_bridge_agent = sil_bridge_agent.sil_bridge_agent:main',
        ],
    },
)
