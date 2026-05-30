from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'phone_imu_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        # Web application files
        (os.path.join('share', package_name, 'webapp'),
            glob('webapp/*.html') + glob('webapp/*.json')),
        (os.path.join('share', package_name, 'webapp', 'css'),
            glob('webapp/css/*.css')),
        (os.path.join('share', package_name, 'webapp', 'js'),
            glob('webapp/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vaibhav',
    maintainer_email='23f3000074@ds.study.iitm.ac.in',
    description=(
        'Phone IMU to ROS 2 bridge with Madgwick fusion, '
        'Butterworth filtering, inertial navigation, and web dashboard'
    ),
    license='MIT',
    tests_require=['pytest'],
    project_urls={
        'Source': 'https://github.com/Bishu-crypto/phone-imu-bridge',
        'Bug Tracker': 'https://github.com/Bishu-crypto/phone-imu-bridge/issues',
    },
    entry_points={
        'console_scripts': [
            'http_receiver     = phone_imu_bridge.http_receiver_node:main',
            'madgwick_filter   = phone_imu_bridge.madgwick_filter_node:main',
            'spectral_analysis = phone_imu_bridge.spectral_analysis_node:main',
            'inertial_nav      = phone_imu_bridge.inertial_nav_node:main',
            'ws_bridge         = phone_imu_bridge.ws_bridge_node:main',
        ],
    },
)
