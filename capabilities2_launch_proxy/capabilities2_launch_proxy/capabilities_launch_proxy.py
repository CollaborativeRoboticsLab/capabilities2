'''
capabilities launch proxy server

implements a launch server responding to messages to launch and shutdown launch files
use in conjunction with capabilities2_server to launch capabilities

acts like ros2 launch command but from a topic
'''

from typing import List
from typing import Mapping
from typing import Text

import threading
import rclpy
from rclpy.node import Node
from launch import LaunchService
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from std_msgs.msg import String
from capabilities2_msgs.msg import CapabilityEvent


class CapabilitiesLaunchProxy(Node):
    """
    Capabilities Launch proxy server
    """

    def __init__(self, node_name='capabilities_launch_proxy'):
        # super class init
        super().__init__(node_name)

        # active contexts list
        self.active_launch_services: List[Mapping[Text, LaunchService]] = []

        # create launch sub
        self.launch_sub = self.create_subscription(
            String,
            '~/launch',
            self.launch_cb,
            10
        )

        # create shutdown sub
        self.shutdown_sub = self.create_subscription(
            String,
            '~/shutdown',
            self.shutdown_cb,
            10
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
    def launch_cb(self, msg: String):
        """
        launch callback
        """

        # launch context already exists guard
        for active_context in self.active_launch_services:
            if active_context[0] == msg.data:
                self.get_logger().error('context already exists for {}'.format(msg.data))
                return

        # create a new service
        service = LaunchService()

        description = LaunchDescription([
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    msg.data
                )
            ),
        ])

        service.include_launch_description(description)

        # bind request to thread
        threading.Thread(target=service.run).start()

        # add context to active contexts
        self.active_launch_services.append([msg.data, service])

    # shutdown sub callback
    def shutdown_cb(self, req: String):
        """
        shutdown callback
        """

        # find the context
        service = None
        for active_context in self.active_launch_services:
            if active_context[0] == req.data:
                service = active_context[1]
                break

        # if context is not found log and return
        if service is None:
            self.get_logger().error('no context found for {}'.format(req.data))
            return

        # shutdown the context
        threading.Thread(target=service.shutdown).start()


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
    for active_context in capabilities_launch_proxy.active_launch_services:
        active_context[1].shutdown()

    capabilities_launch_proxy.destroy_node()
    rclpy.shutdown()

    exit(0)


# main
if __name__ == '__main__':
    main()
