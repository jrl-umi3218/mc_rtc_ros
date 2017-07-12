from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(packages=["mc_sequence_visualization"],
                                      package_dir={'mc_sequence_visualization': 'src/mc_sequence_visualization'})

setup(**setup_args)
