from setuptools import setup
import os
from glob import glob

package_name = 'aruco_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.npz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwj',
    maintainer_email='m6488j@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_recog_node = aruco_rviz.aruco_recognition_node:main',
            'aruco_tf_node = aruco_rviz.tf_broadcast_node:main',
            'aruco_text_marker = aruco_rviz.text_marker_node:main'
        ],
    },
)
