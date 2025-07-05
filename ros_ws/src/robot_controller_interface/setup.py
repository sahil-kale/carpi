from setuptools import find_packages, setup

package_name = 'robot_controller_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + 'msg', ['msg/RobotCommandRaw.msg']),
        ('share/' + package_name + 'msg', ['msg/BuzzerCommand.msg']),
        ('share/' + package_name + 'msg', ['msg/LedCommand.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller_interface_node = robot_controller_interface.robot_controller_interface_node:main',
        ],
    },
)
