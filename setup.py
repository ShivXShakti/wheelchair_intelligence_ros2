from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wheelchair_intelligence_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='kuldeeplakhansons@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'e_intelligence_to_nav2_com = wheelchair_intelligence_ros2.e_intelligence_to_nav2_com:main',
            'e_create_semantics_map = wheelchair_intelligence_ros2.create_semantics_for_map:main',
            'e_send_goal = wheelchair_intelligence_ros2.e_send_goal:main',
        ],
    },
)
