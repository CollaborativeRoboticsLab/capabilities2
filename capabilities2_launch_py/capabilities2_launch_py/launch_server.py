import os
import signal
import rclpy
from launch_process import launch_process
from rclpy.node import Node
from capabilities2_msgs.srv import Launch

class LaunchServer(Node):
    def __init__(self):
        super().__init__('launch_file_server')

        # Service for starting a launch file
        self.start_server = self.create_service(
            Launch,
            '/capabilities/launch_start',
            self.start_request
        )

        # Service for stopping a launch file
        self.stop_server = self.create_service(
            Launch,
            '/capabilities/launch_stop',
            self.stop_request
        )

        # Dictionary to track running processes
        self.processes = {}

        self.get_logger().info('LaunchFileServer is ready.')

    def start_request(self, request, response):
        package_name = request.package_name
        launch_name = request.launch_file_name

        self.get_logger().info(f'Received launch start request: package = {package_name}, launch file = {launch_name}')

        name = package_name + "/" + launch_name

        self.processes[name] = launch_process.LaunchProcess(package_name=package_name, launch_file_name=launch_name)
        self.processes[name].start()

        return response

    def stop_request(self, request, response):
        package_name = request.package_name
        launch_name = request.launch_file_name

        self.get_logger().info(f'Received launch stop request: package = {package_name}, launch file = {launch_name}')

        name = package_name + "/" + launch_name

        self.processes[name].stop()
        self.processes[name].join()

        return response


def main(args=None):
    rclpy.init(args=args)

    # Create the LaunchServer node
    launch_server = LaunchServer()

    # Keep the node running
    try:
        rclpy.spin(launch_server)
    except KeyboardInterrupt:
        launch_server.get_logger().info('Shutting down LaunchFileServer...')
    finally:
        # Clean up all running processes
        for pid, process in launch_server.processes.items():
            os.killpg(os.getpgid(pid), signal.SIGTERM)
            process.wait()

        rclpy.shutdown()


if __name__ == '__main__':
    main()