import os

from setuptools import find_packages, setup

package_name = 'remote_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/remote_agent_params.yaml']),
        ('share/' + package_name + '/config', ['config/gst_pipeline.txt']),
        ('share/' + package_name + '/launch', ['launch/remote_agent.launch.py']),
        ('share/' + package_name + '/scripts', ['scripts/remote-agent.service']),
        # 可执行入口装到 lib/<pkg>/：**不依赖 setuptools 发行版元数据**
        # （console_scripts 入口包装在 --symlink-install 下会抛 PackageNotFoundError，
        #   HUNTER-001 实机因此陷入 activating 重启循环）
        (os.path.join('lib', package_name), ['scripts/remote_agent_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HUNTER Development Team',
    maintainer_email='developer@hunter.ai',
    description='Remote control agent for HUNTER autonomous vehicle system - WebRTC video streaming and command reception',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remote_agent = remote_agent.remote_agent:main',
        ],
    },
)