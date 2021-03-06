from setuptools import setup
import os
from glob import glob

package_name = 'jetbot_btrees'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JJ Reese',
    maintainer_email='jjreese@ncsu.edu',
    description='Jetbot Behavior Trees',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_controller = jetbot_btrees.basic_controller:main',
            'timeout_controller = jetbot_btrees.timeout_controller:main',
        ],
    },
)
