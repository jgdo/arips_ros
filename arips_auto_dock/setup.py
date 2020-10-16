from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['arips_auto_dock'],
    scripts=['scripts/auto_dock'],
    package_dir={'': 'src'}
)

setup(**d)

