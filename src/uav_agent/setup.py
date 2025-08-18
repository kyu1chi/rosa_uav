from setuptools import setup
import os
from glob import glob

package_name = 'uav_agent'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Agent Maintainer',
    maintainer_email='your.email@example.com',
    description='ROSA UAV Agent for PX4 drone control using natural language',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_agent = uav_agent.uav_agent:main',
        ],
    },
)
