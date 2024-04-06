#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['scripts/svo_pose_to_txt.py'],
    packages=['rudpt_svo'],
    package_dir={'': 'src'},
    install_requires=['rospkg', 'yaml']
    )

setup(**d)