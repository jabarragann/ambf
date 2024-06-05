#  ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# d = generate_distutils_setup(
#     packages=['transformations'],
#     scripts=[''],
#     package_dir={'': 'scripts'}
# )

# setup(**d)

import os
from setuptools import setup

if 'AMENT_PREFIX_PATH' in os.environ:
    # ROS2 setup using setuptools
    setup(
        name='transformations',
        version='0.0.0',
        packages=['transformations'],
        package_dir={'': 'src'}
    )
else:
    # ROS1 setup using distutils and catkin_pkg
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['transformations'],
        scripts=[],
        package_dir={'': 'scripts'}
    )
    setup(**d)
