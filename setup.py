# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['mqtt_bridge'],
    package_dir={'': 'src'},
    install_requires=['paho-mqtt', 'inject', 'msgpack-python']
)

setup(**setup_args)
