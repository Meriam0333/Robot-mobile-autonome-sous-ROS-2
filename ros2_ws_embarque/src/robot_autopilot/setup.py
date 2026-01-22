from setuptools import find_packages, setup

package_name = 'robot_autopilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['autopilot_serial = robot_autopilot.autopilot_serial:main',
        'autopilot_mission = robot_autopilot.autopilot_mission:main',
	'auto_simple = robot_autopilot.autopilot_simple:main',
	'autopilot_forward_once = robot_autopilot.autopilot_forward_once:main',
        ],
    },
)
