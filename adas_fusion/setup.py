from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adas_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'pyserial'],
    zip_safe=True,
    maintainer='Moneve',
    maintainer_email='moneve@github.com',
    description='ADAS multi-sensor fusion and decision nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_adapter = adas_fusion.detection_adapter:main',
            'radar_adapter = adas_fusion.radar_adapter:main',
            'fusion_node = adas_fusion.fusion_node:main',
            'decision_node = adas_fusion.decision_node:main',
            'serial_bridge = adas_fusion.serial_bridge:main',
        ],
    },
)
