from setuptools import find_packages
from setuptools import setup

setup(
    name='adaptive_clustering',
    version='0.0.1',
    packages=find_packages(
        include=('adaptive_clustering', 'adaptive_clustering.*')),
)
