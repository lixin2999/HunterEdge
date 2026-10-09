from setuptools import find_packages, setup

package_name = 'hunter_kafka'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HUNTER Development Team',
    maintainer_email='developer@hunter.ai',
    description='HunterCore vehicle-side Kafka access library: kafka.properties single source of '
                'truth, SASL_SSL + mTLS config assembly for librdkafka, and preflight diagnostics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hunter-kafka-check = hunter_kafka.diagnose:main',
        ],
    },
)
