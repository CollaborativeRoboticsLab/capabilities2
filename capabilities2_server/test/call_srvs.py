''' test call cap services '''

import rclpy
from capabilities2_msgs.srv import GetInterfaces
from capabilities2_msgs.srv import GetSemanticInterfaces
from capabilities2_msgs.srv import GetProviders
from capabilities2_msgs.srv import GetCapabilitySpecs
from capabilities2_msgs.srv import EstablishBond

from bondpy import bondpy


# main
if __name__ == '__main__':

    rclpy.init()

    node = rclpy.create_node('test_call_cap_srvs')

    # add service client
    client = node.create_client(
        GetInterfaces,
        '/capabilities/get_interfaces'
    )

    # wait for service
    print('get interfaces')
    client.wait_for_service()

    # send request
    request = GetInterfaces.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    print(future.result())

    # add service client for semantic interfaces
    client = node.create_client(
        GetSemanticInterfaces,
        '/capabilities/get_semantic_interfaces'
    )

    # wait for service
    print('get semantic interfaces')
    client.wait_for_service()

    # send request
    request = GetSemanticInterfaces.Request()
    request.interface = 'std_capabilities/empty'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    print(future.result())

    # get provider for an interface
    client = node.create_client(
        GetProviders,
        '/capabilities/get_providers'
    )

    # wait for service
    print('get providers')
    client.wait_for_service()

    # send request
    request = GetProviders.Request()
    request.interface = 'std_capabilities/empty'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    print(future.result())

    # get cap specs
    client = node.create_client(
        GetCapabilitySpecs,
        '/capabilities/get_capability_specs'
    )

    # wait for service
    client.wait_for_service()

    # send request
    request = GetCapabilitySpecs.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    print(future.result())

    # try establish bond
    # add service client
    client = node.create_client(
        EstablishBond,
        '/capabilities/establish_bond'
    )

    # wait for service
    client.wait_for_service()

    # send request
    request = EstablishBond.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    print(future.result())

    # try maintain bond for a bit
    bond = bondpy.Bond(node, "/capabilities/bonds", future.result().bond_id)
    bond.start()

    # Do some work while the bond is active
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    exit(0)
