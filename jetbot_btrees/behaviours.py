#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviours for the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

##############################################################################
# Behaviours
##############################################################################


class MotorControl(py_trees.behaviour.Behaviour):
    def __init__(
            self,
            name: str,
            topic_name: str="jetbot_motor_twist",
    ):
        super(MotorControl, self).__init__(name=name)
        self.topic_name = topic_name

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.
        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree
        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher = self.node.create_publisher(
            msg_type=geometry_msgs.Twist,
            topic="jetbot_motor_twist",
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

        self.feedback_message = "publisher created"
        self.blackboard = py_trees.blackboard.Client(name="ControlClient")
        self.blackboard.register_key(key="sensor_dist", access=py_trees.common.Access.READ)
        self.p_gain = 7
        self.max_speed = 1.0

    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % self.__class__.__name__)
        twist_msg = geometry_msgs.Twist()
        speed = self.blackboard.sensor_dist * self.p_gain
        twist_msg.linear.x = speed if speed < self.max_speed else self.max_speed
        self.publisher.publish(twist_msg)
        self.feedback_message = f"Sending Forward Command, speed: {speed if speed < self.max_speed else self.max_speed}"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.
        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)
        self.feedback_message = "cleared"


class MotorStop(py_trees.behaviour.Behaviour):

    def __init__(
            self,
            name: str,
            topic_name: str="jetbot_motor_twist"
    ):
        super(MotorStop, self).__init__(name=name)
        self.topic_name = topic_name

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher = self.node.create_publisher(
            msg_type=geometry_msgs.Twist,
            topic="jetbot_motor_twist",
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

        self.feedback_message = "publisher created"

    def update(self) -> py_trees.common.Status:

        self.logger.debug("%s.update()" % self.__class__.__name__)
        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)
        self.feedback_message = "Stopping Motors!"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)
        self.feedback_message = "cleared"

