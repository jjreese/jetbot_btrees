#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Jetbot Collision Launcher
"""
##############################################################################
# Imports
##############################################################################

import jetbot_btrees_pkg.jetbot_collision as jetbot_launch

##############################################################################
# Launch Service
##############################################################################


def generate_launch_description():
    """
    Launch description for the tutorial.
    """
    return jetbot_launch.generate_launch_description()
