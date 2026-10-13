import subprocess
import sys

import rclpy
from rclpy.node import Node


class TestProcessTree(Node):
    def __init__(self):
        super().__init__('capabilities2_launch_proxy_test_process_tree')
        self.child_process = subprocess.Popen([
            sys.executable,
            '-c',
            'import time; time.sleep(60)',
        ])
        self.timer = self.create_timer(0.2, self.publish_heartbeat)

    def publish_heartbeat(self):
        self.get_logger().info('process tree heartbeat')

    def destroy_node(self):
        if self.child_process.poll() is None:
            self.child_process.terminate()
            try:
                self.child_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.child_process.kill()
                self.child_process.wait(timeout=5.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = TestProcessTree()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()