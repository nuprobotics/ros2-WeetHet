from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@contextmanager
def rclpy_context(args=None):
    rclpy.init(args=args)
    try:
        yield
    finally:
        rclpy.shutdown()


class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")

        self.declare_parameter("topic_name", "/spgc/receiver")
        self.declare_parameter("text", "Hello, ROS2!")

        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        text = self.get_parameter("text").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, topic_name, 10)

        msg = String()
        msg.data = text

        timer_period = 1.0
        self.msg = msg
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Publishing: "{self.msg.data}"')


def main(args=None):
    with rclpy_context(args=args):
        node = PublisherNode()
        rclpy.spin(node)
        node.destroy_node()


if __name__ == "__main__":
    main()
