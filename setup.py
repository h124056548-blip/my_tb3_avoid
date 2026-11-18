import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_tb3_avoid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 加上下面這行，將所有 .launch.py 檔案安裝到套件的 share/launch 目錄中
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='h1240',
    maintainer_email='h1240@todo.todo',
    description='TurtleBot3 autonomous patrol and obstacle avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_node = my_tb3_avoid.obstacle_avoid:main',
            'goal_node = my_tb3_avoid.go_to_goal_avoid:main',
            'patrol_node = my_tb3_avoid.waypoint_patrol:main',
        ],
    },
)
