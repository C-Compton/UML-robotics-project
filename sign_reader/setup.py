from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sign_reader'],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
