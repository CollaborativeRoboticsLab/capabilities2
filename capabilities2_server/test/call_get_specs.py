''' test call cap services '''

import rclpy
from capabilities2_msgs.srv import GetCapabilitySpecs


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


# main
if __name__ == '__main__':

    rclpy.init()

    node = rclpy.create_node('test_call_cap_srvs')

    # do tests
    # test_get_interfaces_srv(node)
    # test_get_semantic_interfaces_srv(node)
    # test_get_providers_srv(node)
    test_get_capability_specs_srv(node)

    rclpy.shutdown()

    exit(0)
