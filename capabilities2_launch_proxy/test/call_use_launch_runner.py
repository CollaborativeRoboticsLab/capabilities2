''' test call cap services '''

from bondpy import bondpy
import rclpy
from capabilities2_msgs.srv import EstablishBond
from capabilities2_msgs.srv import GetRunningCapabilities
from capabilities2_msgs.srv import UseCapability


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
    # keep bond id
    bond_id = future.result().bond_id

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

    # keep bond alive
    bond = bondpy.Bond(node, "/capabilities/bonds", bond_id)
    bond.start()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    exit(0)
