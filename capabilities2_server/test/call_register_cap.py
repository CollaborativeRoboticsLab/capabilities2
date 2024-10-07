''' test call cap services '''

import rclpy
from capabilities2_msgs.srv import RegisterCapability


def test_register_cap(n):
    # try establish bond
    # add service client
    client = n.create_client(
        RegisterCapability,
        '/capabilities/register_capability'
    )

    # wait for service
    client.wait_for_service()

    # send request
    request: RegisterCapability.Request = RegisterCapability.Request()
    # fill request
    request.capability_spec.package = 'std_capabilities'
    request.capability_spec.type = 'capability_provider'
    request.capability_spec.content = """
    name: test_cap_server_provider
    spec_type: provider
    spec_version: 1.1
    description: capability server test provider
    implements: std_capabilities/empty
    runner: capabilities2_runner::DummyRunner
    """
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future)

    # print result
    print(future.result())


# main
if __name__ == '__main__':

    rclpy.init()

    node = rclpy.create_node('test_call_cap_srvs')

    # do tests
    test_register_cap(node)

    rclpy.shutdown()

    exit(0)
