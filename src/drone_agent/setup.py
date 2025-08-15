from setuptools import setup, find_packages

package_name = 'drone_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/agent.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'mavsdk>=1.4.0',
        'jpl-rosa>=1.0.7',
        'langchain>=0.1.0',
        'langchain-openai>=0.1.0',
        'rich>=13.0.0',
        'python-dotenv>=1.0.0',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROSA-based drone control agent for ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_agent = drone_agent.scripts.drone_agent:main',
        ],
    },
)