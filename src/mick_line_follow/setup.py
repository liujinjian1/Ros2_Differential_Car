from setuptools import setup, find_packages
import os
from glob import glob 

package_name = 'mick_line_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=('mick_line_follow', 'mick_line_follow.*')),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mickrobot',
    maintainer_email='mickrobot@todo.todo',
    description='Line following robot control package',  
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_node = mick_line_follow.line_follow:main',
        ],
    },
)
