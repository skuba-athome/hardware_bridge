#!/usr/bin/env python
from subprocess import call
import getpass
import roslib

# add user to dialout group
username = getpass.getuser()
call(["sudo", "adduser", username, 'dialout'])

# copy rule to udev folder
hardware_bridge_path = roslib.packages.get_pkg_dir('hardware_bridge')
src = "{0}/99-skuba-usb.rules".format(hardware_bridge_path)
dest = "/etc/udev/rules.d/99-skuba-usb.rules"
call(["sudo", "cp", src, dest])

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hardware_bridge'],
    package_dir={'': 'src'},
    )

setup(**d)
