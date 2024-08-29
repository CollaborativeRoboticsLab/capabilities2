'''
capabilities launch proxy server

implements a launch server responding to messages to launch and shutdown launch files
use in conjunction with capabilities2_server to launch capabilities

acts like ros2 launch command but from a topic
'''

from typing import Dict
from typing import Text

import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from launch import LaunchService
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from capabilities2_msgs.action import Launch
from capabilities2_msgs.msg import CapabilityEvent


class CapabilitiesLaunchProxy(Node):
    """
    Capabilities Launch proxy server
    """

    def __init__(self, node_name='capabilities_launch_proxy'):
        # super class init
        super().__init__(node_name)

        # active contexts list
        self.active_launch_services: Dict[Text, LaunchService] = {}

        # create launch action
        self.launch_sub = ActionServer(
            self,
            Launch,
            '~/launch',
            execute_callback=self.launch_cb,
            cancel_callback=self.shutdown_cb
        )

        # event pub
        self.event_pub = self.create_publisher(
            CapabilityEvent,
            '~/events',
            10
        )

        # log start
        self.get_logger().info('capabilities launch proxy started')

    # launch sub callback
    def launch_cb(self, msg: Launch):
        """
        launch callback
        """

        # launch context already exists guard
        for active_context in self.active_launch_services:
            if active_context[0] == msg.launch_file_path:
                self.get_logger().error('context already exists for {}'.format(msg.launch_file_path))
                return

        # create a new service
        service = LaunchService()

        description = LaunchDescription([
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    msg.launch_file_path
                )
            ),
        ])

        service.include_launch_description(description)

        # bind request to thread
        threading.Thread(target=service.run).start()

        # send event
        event = CapabilityEvent()
        event.event = 'launch'
        event.data = msg.launch_file_path
        self.event_pub.publish(event)

        # add context to active contexts
        self.active_launch_services[msg.launch_file_path] = service

    # shutdown sub callback
    def shutdown_cb(self, goal_handle) -> CancelResponse:
        """
        shutdown callback (cancel goal)
        """

        # if context is not found log and return
        if self.active_launch_services[] is None:
            self.get_logger().error('no context found for {}'.format(req.data))
            return CancelResponse.REJECT

        # find the context and
        # shutdown the context
        threading.Thread(target=self.active_launch_services[].shutdown).start()

        return CancelResponse.ACCEPT


# main function
def main():
    '''
    main function
    '''

    # init node
    rclpy.init()

    # instantiate the capabilities launch server
    capabilities_launch_proxy = CapabilitiesLaunchProxy()

    # spin
    rclpy.spin(capabilities_launch_proxy)

    # cancel the launch services
    for k, active_context in capabilities_launch_proxy.active_launch_services:
        active_context.shutdown()

    capabilities_launch_proxy.destroy_node()
    rclpy.shutdown()

    exit(0)


# main
if __name__ == '__main__':
    main()
