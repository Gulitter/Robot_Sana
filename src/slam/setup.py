
import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.lua'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.STL'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guyechan',
    maintainer_email='guyechan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = slam.main:main',
        ],
    },
)
