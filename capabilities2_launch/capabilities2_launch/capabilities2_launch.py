import os
import signal
import rclpy
from rclpy.node import Node
from capabilities2_msgs.srv import Launch
from multiprocessing.connection import Client

class LaunchServer(Node):
    def __init__(self):
        super().__init__('capabilities2_launch_server')
        self.address = ('localhost', 6000)
        self.client  = Client(self.address, authkey=b'capabilities2')

        # Service for starting a launch file
        self.start_server = self.create_service(
            Launch,
            '/capabilities/launch/start',
            self.start_request
        )

        # Service for stopping a launch file
        self.stop_server = self.create_service(
            Launch,
            '/capabilities/launch/stop',
            self.stop_request
        )

        # Service for stopping a launch file
        self.stop_server = self.create_service(
            Launch,
            '/capabilities/launch/status',
            self.status_request
        )

        self.get_logger().info('Capabilities2 LaunchServer is ready.')

    def start_request(self, request, response):
        package_name = request.package_name
        launch_file_name = request.launch_file_name

        self.get_logger().info(f'[{package_name}/{launch_file_name}] received start request')

        message_outgoing = {}
        message_outgoing["command"] = "start"
        message_outgoing["package"] = package_name
        message_outgoing["launch_file"] = launch_file_name

        self.get_logger().info(f'[{package_name}/{launch_file_name}] sending start request to Launch Server')

        self.client.send(message_outgoing)

        self.get_logger().info(f'[{package_name}/{launch_file_name}] waiting for response from Launch Server')

        message_incoming = self.client.recv()
        response_content = message_incoming["status"]

        self.get_logger().info(f'[{package_name}/{launch_file_name}] received response : {response_content}')

        response.status = response_content

        return response

    def stop_request(self, request, response):
        package_name = request.package_name
        launch_file_name = request.launch_file_name

        self.get_logger().info(f'[{package_name}/{launch_file_name}] received stop request')

        message_outgoing = {}
        message_outgoing["command"] = "stop"
        message_outgoing["package"] = package_name
        message_outgoing["launch_file"] = launch_file_name

        self.get_logger().info(f'[{package_name}/{launch_file_name}] sending stop request to Launch Server')

        self.client.send(message_outgoing)

        self.get_logger().info(f'[{package_name}/{launch_file_name}] waiting for response from Launch Server')

        message_incoming = self.client.recv()
        response_content = message_incoming["status"]

        self.get_logger().info(f'[{package_name}/{launch_file_name}] received response : {response_content}')

        response.status = response_content

        return response

    
    def status_request(self, request, response):
        package_name = request.package_name
        launch_file_name = request.launch_file_name

        self.get_logger().info(f'[{package_name}/{launch_file_name}] received status request')

        message_outgoing = {}
        message_outgoing["command"] = "status"
        message_outgoing["package"] = package_name
        message_outgoing["launch_file"] = launch_file_name

        self.get_logger().info(f'[{package_name}/{launch_file_name}] sending status request to Launch Server')

        self.client.send(message_outgoing)

        self.get_logger().info(f'[{package_name}/{launch_file_name}] waiting for response from Launch Server')

        message_incoming = self.client.recv()
        response_content = message_incoming["status"]

        self.get_logger().info(f'[{package_name}/{launch_file_name}] received response : {response_content}')

        response.status = response_content

        return response
    
    def shutdown(self):

        self.get_logger().info(f'Shutting down LaunchFileServer...')

        message_outgoing = {}
        message_outgoing["command"] = "exit"
        message_outgoing["package"] = ""
        message_outgoing["launch_file"] = ""

        self.get_logger().info(f'[sending shutdown request to Launch Server')

        self.client.send(message_outgoing)


def main(args=None):
    rclpy.init(args=args)

    # Create the LaunchServer node
    launch_server = LaunchServer()

    # Keep the node running
    try:
        rclpy.spin(launch_server)
    except KeyboardInterrupt:
        launch_server.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()