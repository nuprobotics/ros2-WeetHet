import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Receiver(Node):
    def __init__(self):
        super().__init__("receiver")
        self.subscription = self.create_subscription(
            String, "/spgc/sender", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    with rclpy.init(args=args):
        node = Receiver()
        rclpy.spin(node)
        node.destroy_node()


if __name__ == "__main__":
    main()
