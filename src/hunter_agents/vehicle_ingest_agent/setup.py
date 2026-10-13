import os

from setuptools import find_packages, setup

package_name = 'vehicle_ingest_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vehicle_ingest_agent.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/vehicle_ingest_agent.launch.py']),
        ('share/' + package_name + '/systemd',
         ['systemd/vehicle-ingest-agent.service']),
        # 可执行入口装到 lib/<pkg>/：**不依赖 setuptools 发行版元数据**
        # （console_scripts 入口包装在 --symlink-install 下会因 easy-install.pth 不生效
        #   抛 PackageNotFoundError；见 command_agent/remote_agent 同款修复）。
        (os.path.join('lib', package_name), ['scripts/vehicle_ingest_agent_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HUNTER Development Team',
    maintainer_email='developer@hunter.ai',
    description='Vehicle rosbag ingest agent: ring-buffer segmented recording, manifest metadata, '
                'safe raw-bag upload and disk governance (parse/slice/structure on server)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_ingest_agent = vehicle_ingest_agent.agent:main',
        ],
    },
)
