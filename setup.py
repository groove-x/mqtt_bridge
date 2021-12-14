from setuptools import setup
import os
from glob import glob

package_name = 'mqtt_bridge'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kapil Deshpande & Junya Hayashi',
    maintainer_email='kapildeshpande041193@gmail.com & junya.hayashi@groove-x.com',
    description='The mqtt_bridge package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = mqtt_bridge.mqtt_bridge_node:main'
        ],
    },
)
