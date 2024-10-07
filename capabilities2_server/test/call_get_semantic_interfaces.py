''' test call cap services '''

import rclpy
from capabilities2_msgs.srv import GetSemanticInterfaces


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


# main
if __name__ == '__main__':

    rclpy.init()

    node = rclpy.create_node('test_call_cap_srvs')

    # do tests
    # test_get_interfaces_srv(node)
    test_get_semantic_interfaces_srv(node)
    # test_get_providers_srv(node)
    # test_get_capability_specs_srv(node)

    rclpy.shutdown()

    exit(0)
