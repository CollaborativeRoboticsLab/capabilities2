''' test call cap services '''

from bondpy import bondpy
import rclpy
from capabilities2_msgs.srv import GetInterfaces
from capabilities2_msgs.srv import GetSemanticInterfaces
from capabilities2_msgs.srv import GetProviders
from capabilities2_msgs.srv import GetCapabilitySpecs
from capabilities2_msgs.srv import EstablishBond
from capabilities2_msgs.srv import GetRunningCapabilities
from capabilities2_msgs.srv import UseCapability


def test_get_interfaces_srv(n):
    # add service client
    client = n.create_client(
        GetInterfaces,
        '/capabilities/get_interfaces'
    )

    # wait for service
    print('get interfaces')
    client.wait_for_service()

    # send request
    request = GetInterfaces.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future)

    # print result
    print(future.result())


def test_get_semantic_interfaces_srv(n):
    # add service client for semantic interfaces
    client = n.create_client(
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
    rclpy.spin_until_future_complete(n, future)

    # print result
    print(future.result())


def test_get_providers_srv(n):
    # get provider for an interface
    client = n.create_client(
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
    rclpy.spin_until_future_complete(n, future)

    # print result
    print(future.result())


def test_get_capability_specs_srv(n):
    # get cap specs
    client = n.create_client(
        GetCapabilitySpecs,
        '/capabilities/get_capability_specs'
    )

    # wait for service
    client.wait_for_service()

    # send request
    request = GetCapabilitySpecs.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future)

    # print result
    for s in future.result().capability_specs:
        print(s.package + '-' + s.type)
        print(s.content)


def test_get_running_capabilities(n):
    # get running capabilities
    client = n.create_client(
        GetRunningCapabilities,
        '/capabilities/get_running_capabilities'
    )

    # wait for service
    client.wait_for_service()

    # send request
    request = GetRunningCapabilities.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future)

    # print result
    for s in future.result().running_capabilities:
        print(s)


def test_establish_bond(n):
    # try establish bond
    # add service client
    client = n.create_client(
        EstablishBond,
        '/capabilities/establish_bond'
    )

    # wait for service
    client.wait_for_service()

    # send request
    request = EstablishBond.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future)

    # print result
    print(future.result())

    # try maintain bond for a bit
    bond = bondpy.Bond(n, "/capabilities/bonds", future.result().bond_id)
    bond.start()

    # Do some work while the bond is active
    rclpy.spin(n)


# main
if __name__ == '__main__':

    rclpy.init()

    node = rclpy.create_node('test_call_cap_srvs')

    # do tests
    # test_get_interfaces_srv(node)
    # test_get_semantic_interfaces_srv(node)
    # test_get_providers_srv(node)
    # test_get_capability_specs_srv(node)

    print("test use capability")
    # get a bond
    bond_cli = node.create_client(
        EstablishBond,
        '/capabilities/establish_bond'
    )

    # wait for service
    bond_cli.wait_for_service()

    # send request
    request = EstablishBond.Request()
    future = bond_cli.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    print(future.result())

    # create a use capability request
    use_client = node.create_client(
        UseCapability,
        '/capabilities/use_capability'
    )

    # wait for service
    use_client.wait_for_service()

    # send request
    request = UseCapability.Request()
    request.capability = 'std_capabilities/empty'
    request.preferred_provider = 'std_capabilities/empty'
    request.bond_id = future.result().bond_id
    future = use_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    print(future.result())

    # get running capabilities
    test_get_running_capabilities(node)

    node.destroy_node()
    rclpy.shutdown()

    exit(0)
