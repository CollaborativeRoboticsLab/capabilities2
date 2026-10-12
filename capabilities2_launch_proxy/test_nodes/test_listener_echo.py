import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestListenerEcho(Node):
    def __init__(self):
        super().__init__('capabilities2_launch_proxy_test_listener_echo')
        self.publisher = self.create_publisher(String, '/capabilities2_launch_proxy/test_chatter_echo', 10)
        self.subscription = self.create_subscription(
            String,
            '/capabilities2_launch_proxy/test_chatter',
            self.handle_message,
            10,
        )

    def handle_message(self, message: String) -> None:
        response = String()
        response.data = f'echo:{message.data}'
        self.publisher.publish(response)


def main():
    rclpy.init()
    node = TestListenerEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()