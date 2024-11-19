import os
import signal
import subprocess
import rclpy
import copy
from rclpy.node import Node
from capabilities2_msgs.srv import LaunchStart, LaunchStop

class LaunchServer(Node):
    def __init__(self):
        super().__init__('launch_file_server')

        # Service for starting a launch file
        self.start_server = self.create_service(
            LaunchStart,
            '/capabilities/launch_start',
            self.start_request
        )

        # Service for stopping a launch file
        self.stop_server = self.create_service(
            LaunchStop,
            '/capabilities/launch_stop',
            self.stop_request
        )

        # Dictionary to track running processes
        self.processes = {}

        self.get_logger().info('LaunchFileServer is ready.')

    def start_request(self, request, response):
        package_name = request.package_name
        launch_name = request.launch_file_name

        self.get_logger().info(f'Received request: package = {package_name}, launch file = {launch_name}')

        # Construct the command to execute
        command = f'/bin/bash -c "source install/setup.bash && ros2 launch {package_name} {launch_name}"'

        # Start the process
        try:
            env = copy.deepcopy(os.environ)
            env['PYTHONUNBUFFERED'] = 'x'
            process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid, close_fds=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
            self.processes[process.pid] = process  # Track the process

            self.get_logger().info(f'Started {launch_name} from {package_name} with PID: {process.pid}')
            response.pid = process.pid
        except Exception as e:
            self.get_logger().error(f'Failed to start {launch_name} from {package_name}: {str(e)}')
            response.pid = -1

        return response

    def stop_request(self, request, response):
        pid = request.pid
        self.get_logger().info(f'Received request: stopping PID = {pid}')

        if pid in self.processes:
            try:
                # Kill the process group to stop all child processes
                os.killpg(os.getpgid(pid), signal.SIGTERM)
                self.processes[pid].wait()  # Wait for the process to terminate
                del self.processes[pid]
                self.get_logger().info(f'Successfully stopped process with PID: {pid}')
            except Exception as e:
                self.get_logger().warn(f'Failed to terminate PID {pid}: {str(e)}')
        else:
            self.get_logger().warn(f'No process found with PID {pid}')

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