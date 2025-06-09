from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'neupan_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml*'))),
        (os.path.join('share', package_name, 'weight'), glob(os.path.join('weight', '*.pth*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*rviz*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dangmu',
    maintainer_email='changmin5941@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neupan_node=neupan_ros.neupan_node:main',
        ],
    },
)
