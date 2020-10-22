#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################


##############################################################################
# Imports
##############################################################################

import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys

from . import mock




##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Here is where we layout the structure of our tree, creating the root and attaching children to it.
    Returns:
        the root of the tree
    """
	
    int_sec = py_trees.composites.Sequence(
        name="->"
    )
    left = py_trees.composites.Selector(
        name= '?'#"Global Planning"
    )
	
    mid = py_trees.composites.Selector(
        name= '?'#"Local Planning"
    )
	
    root = py_trees.composites.Selector(
        name= 'Marathon2 Behavior Tree'#"Root"
    )
	
    right = py_trees.composites.Selector(
        name= '?'#"Fallback"
    )
	
	
	
    a_star = py_trees.behaviours.Periodic(name = "A* Planner", n = 2)
    teb	= py_trees.behaviours.Periodic(name = "TEB Controller", n = 2)
    clear_all =  py_trees.behaviours.Failure(name="Clear All Environment") 
    clear_local =  py_trees.behaviours.Failure(name="Clear Local Environment")           
    clear_glob =  py_trees.behaviours.Failure(name="Clear Global Environment")    
    spin =  py_trees.behaviours.Failure(name="Spin")    
    wait =  py_trees.behaviours.Success(name="Wait")    

    left.add_child(a_star)
    left.add_child(clear_glob)
    mid.add_child(teb)
    mid.add_child(clear_local)
    int_sec.add_child(left)
    int_sec.add_child(mid)
    right.add_child(clear_all)
    right.add_child(spin)
    right.add_child(wait)
	
    root.add_child(int_sec)
    root.add_child(right)
    

    return root


def tutorial_main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=500.0) # ms = Miliseconds

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()
