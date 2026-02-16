from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


@contextmanager
def rclpy_context(args=None):
    rclpy.init(args=args)
    try:
        yield
    finally:
        rclpy.shutdown()


class TriggerNode(Node):
    def __init__(self):
        super().__init__("trigger_node")

        self.declare_parameter("service_name", "/trigger_service")
        self.declare_parameter("default_string", "No service available")

        service_name = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )

        self.default_string = (
            self.get_parameter("default_string").get_parameter_value().string_value
        )

        self.stored_response = None
        self.client = self.create_client(Trigger, "/spgc/trigger")
        self.service = self.create_service(Trigger, service_name, self.service_callback)
        self.call_trigger_service()

    def call_trigger_service(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/spgc/trigger service not available")
            self.stored_response = None
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.trigger_response_callback)

    def trigger_response_callback(self, future):
        try:
            response = future.result()
            self.stored_response = response.message
            self.get_logger().info(
                f"Received response from /spgc/trigger: {self.stored_response}"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.stored_response = None

    def service_callback(self, request, response):
        match self.stored_response:
            case None:
                response.success = False
                response.message = self.default_string
            case _:
                response.success = True
                response.message = self.stored_response
        return response


def main(args=None):
    with rclpy_context(args=args):
        node = TriggerNode()
        rclpy.spin(node)
        node.destroy_node()


if __name__ == "__main__":
    main()
