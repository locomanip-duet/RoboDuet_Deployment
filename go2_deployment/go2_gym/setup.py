from setuptools import find_packages
from distutils.core import setup

setup(
    name='go2_gym',
    version='1.0.0',
    author='Guoping Pan',
    license="BSD-3-Clause",
    packages=find_packages(),
    author_email='panguoping02@gmail.com',
    description='Toolkit for deployment of sim-to-real RL on the Unitree Go2+ARX.'
)