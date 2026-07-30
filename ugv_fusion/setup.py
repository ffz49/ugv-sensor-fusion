import os
from glob import glob
from setuptools import setup

package_name = 'ugv_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. This tells the build system to install your launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='felix909377@gmail.com',
    description='Master thesis UGV sensor fusion pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 2. This registers your scripts so they can be run by ROS 2
            'visual_bev_node = ugv_fusion.visual_bev_node:main',
            'radar_bev_node = ugv_fusion.radar_bev_node:main',
            'radar_clarity_node = ugv_fusion.radar_clarity_node:main',
            'radar_object_node = ugv_fusion.radar_object_node:main',
            'visual_clarity_node = ugv_fusion.visual_clarity_node:main',
            'dempster_shafer_fusion_node = ugv_fusion.dempster_shafer_fusion_node:main',
            'spatio_temporal_association_node = ugv_fusion.spatio_temporal_association_node:main',
            # ADDED: Your new experiment control node
            'experiment_control_node = ugv_fusion.experiment_control_node:main',
        ],
    },
)
