from distutils.core import setup
from setuptools import find_packages
setup(name='urlibs',
      version='1.0',
      package_dir= {'': 'src'},
      packages=find_packages('src'),
      )
