import rclpy      # ROS CLIENT LIBRARIES
from rclpy.node import Node
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
#from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Range, 'tof_distance', 10)
        timer_period = (1)  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = Range()
        self.msg.range = 30.0

    def timer_callback(self):
        self.msg.range -= 1
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: "%s"' % self.msg.range)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
