from setuptools import find_packages, setup

package_name = 'command_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/command_agent_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/command_agent.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HUNTER Development Team',
    maintainer_email='developer@hunter.ai',
    description='HunterCore command agent: consume hunter.<vehicle_id>.command, map to existing '
                'ROS2 services/topics and report hunter.<vehicle_id>.command_result',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_agent = command_agent.command_agent:main',
        ],
    },
)
