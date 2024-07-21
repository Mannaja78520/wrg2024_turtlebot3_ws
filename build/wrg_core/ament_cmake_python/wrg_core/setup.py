from setuptools import find_packages
from setuptools import setup

setup(
    name='wrg_core',
    version='2.1.5',
    packages=find_packages(
        include=('wrg_core', 'wrg_core.*')),
)
