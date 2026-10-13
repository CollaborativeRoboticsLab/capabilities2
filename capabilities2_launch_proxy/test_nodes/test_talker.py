import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestTalker(Node):
    def __init__(self):
        super().__init__('capabilities2_launch_proxy_test_talker')
        self.publisher = self.create_publisher(String, '/capabilities2_launch_proxy/test_chatter', 10)
        self.timer = self.create_timer(0.2, self.publish_message)

    def publish_message(self):
        message = String()
        message.data = 'hello-from-capabilities2-launch'
        self.publisher.publish(message)
        self.get_logger().info('published test message')


def main():
    rclpy.init()
    node = TestTalker()
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