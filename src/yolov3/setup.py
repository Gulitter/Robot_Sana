import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'yolov3'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.cfg'))),
        (os.path.join('share', package_name, 'weights'), glob(os.path.join('weights', '*.weights'))),
        (os.path.join('share', package_name, 'parameter'), glob(os.path.join('parameter', '*.names')))
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
            'yolov3 = yolov3.yolov3:main'
        ],
    },
)
