from os.path import join, dirname, realpath
from setuptools import setup
import sys

setup(
    name='cairo_simulator',
    py_modules=['cairo_simulator'],
    version="1.0",
    install_requires=[
        'pybullet',
        'rospy',
        'numpy',
    ],
    description="PyBullet-based Robot Simulator maintained by CAIRO Lab."
)