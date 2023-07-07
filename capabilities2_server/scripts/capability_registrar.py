#!/usr/bin/env python

# Capability Registrar Node
# a ros node that registers the capabilities of the robot to the capability server
# the registration is done by calling the capability server's service

import rospy


# registrar class
class CapabilityRegistrar:
    def __init__(self) -> None:
        pass

    # register the capabilities
    def register_capabilities(self):
        pass


# main
if __name__ == '__main__':
    # init node
    rospy.init_node('capability_registrar_node')

    rospy.loginfo('capability_registrar_node started')

    # instantiate the registrar
    registrar = CapabilityRegistrar()

    # register the given capabilities
    registrar.register_capabilities()

    # done
    rospy.loginfo('capability_registrar_node finished')

    # exit
    exit(0)
