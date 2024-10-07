''' test call cap services '''

from bondpy import bondpy
import rclpy
from capabilities2_msgs.srv import EstablishBond


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
    test_establish_bond(node)

    rclpy.shutdown()

    exit(0)
