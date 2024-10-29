#!/usr/bin/env python
'''
 Capability Registrar Node
 a ros node that registers the capabilities of the robot to the capability server
 the registration is done by calling the capability server's service
'''


import argparse
import yaml
import rclpy
from rclpy.node import Node
from capabilities2_msgs.srv import RegisterCapability


# registrar class
class CapabilityRegistrar(Node):
    """
    Capability Registrar Node
    """

    def __init__(self) -> None:
        super().__init__('capability_registrar')

        # wait for service
        self.get_logger().info('waiting for capability server...')
        self.register_client = self.create_client(
            RegisterCapability, '/capabilities/register_capability')
        self.register_client.wait_for_service()

    # register the capabilities
    def register_capabilities(self, req: RegisterCapability.Request):
        """
        call register service with request
        """
        future = self.register_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)


# main
if __name__ == '__main__':
    # init node
    rclpy.init(args=None)
    # instantiate the registrar
    registrar = CapabilityRegistrar()

    # argument parser
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'file', type=str, help='the capability spec file to register')
    # optional arguments
    parser.add_argument(
        '-p', type=str, help='the package name of the capability spec file', default='registered_capabilities', required=False)
    args = parser.parse_args()

    # get arguments
    # the request
    request = RegisterCapability.Request()
    # a generic package for registering capabilities
    request.capability_spec.package = 'registered_capabilities'

    # the yaml file to register
    filename = args.file
    spec = yaml.safe_load(filename)

    request.capability_spec.type = spec['spec_type']
    request.capability_spec.content = yaml.dump(spec)

    # register the given capability spec
    registrar.register_capabilities(request)

    # done
    registrar.get_logger.info('capability registrar done')
    rclpy.shutdown()

    # exit
    exit(0)
