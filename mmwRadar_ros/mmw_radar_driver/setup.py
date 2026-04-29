from setuptools import setup

package_name = 'mmw_radar_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moneve',
    maintainer_email='moneve@github.com',
    description='ROS 2 driver for MS60-3015S80M4 mmWave radar (AT6010 SOC)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mmw_radar_node = mmw_radar_driver.radar_node:main',
        ],
    },
)
