#!/usr/bin/env python

# Capability Registrar Node
# a ros node that registers the capabilities of the robot to the capability server
# the registration is done by calling the capability server's service

# imports
import rclpy
import rclpy.node as Node
from capabilities2_msgs.srv import RegisterCapability


# registrar class
class CapabilityRegistrar(Node):
    """Capability Registrar Node
    """

    def __init__(self) -> None:
        super().__init__('capability_registrar')

        # wait for service
        self.get_logger().info('waiting for capability server...')
        self.register_client = self.create_client(
            RegisterCapability, 'register_capability')

    # register the capabilities
    def register_capabilities(self):
        pass


# main
if __name__ == '__main__':
    # init node
    rclpy.init()
    # instantiate the registrar
    registrar = CapabilityRegistrar()

    # register the given capabilities
    registrar.register_capabilities()

    # done
    registrar.get_logger.info('capability registrar done')
    rclpy.shutdown()

    # exit
    exit(0)
