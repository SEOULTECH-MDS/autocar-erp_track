from setuptools import find_packages
from setuptools import setup

setup(
    name='delaunay_path_planning',
    version='0.0.0',
    packages=find_packages(
        include=('delaunay_path_planning', 'delaunay_path_planning.*')),
)
