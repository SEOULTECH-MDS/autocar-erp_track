from setuptools import find_packages
from setuptools import setup

setup(
    name='erp_control',
    version='0.0.0',
    packages=find_packages(
        include=('erp_control', 'erp_control.*')),
)
