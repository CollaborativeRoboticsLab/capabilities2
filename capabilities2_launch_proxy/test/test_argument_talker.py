import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestArgumentTalker(Node):
    def __init__(self, message_text: str):
        super().__init__('capabilities2_launch_proxy_test_argument_talker')
        self.message_text = message_text
        self.publisher = self.create_publisher(String, '/capabilities2_launch_proxy/argument_test_chatter', 10)
        self.timer = self.create_timer(0.2, self.publish_message)

    def publish_message(self):
        message = String()
        message.data = self.message_text
        self.publisher.publish(message)
        self.get_logger().info(f'published argument test message: {self.message_text}')


def main():
    message_text = sys.argv[1] if len(sys.argv) > 1 else 'default-launch-message'
    rclpy.init()
    node = TestArgumentTalker(message_text)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()