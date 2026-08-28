from setuptools import find_packages, setup

package_name = 'ota_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ota_agent_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/ota_agent.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HUNTER Development Team',
    maintainer_email='developer@hunter.ai',
    description='OTA upgrade management agent for HUNTER autonomous vehicle system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)