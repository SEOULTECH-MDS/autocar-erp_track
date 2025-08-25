from setuptools import find_packages
from setuptools import setup

setup(
    name='autocar_utils',
    version='0.0.0',
    packages=find_packages(
        include=('autocar_utils', 'autocar_utils.*')),
)
