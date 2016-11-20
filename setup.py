# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['mqtt_bridge'],
    package_dir={'': 'src'},
    requires=['paho', 'inject']
)

setup(**setup_args)
