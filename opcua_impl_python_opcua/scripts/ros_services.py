#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import sys
import time

import rospy
import rosservice
from opcua import Server

global server
global servicesDict


class OpcUaROSService:
    global server

    def __init__(self, parent, idx, service_name, service_class):
        self.name = service_name
        self._class = service_class
        self.proxy = rospy.ServiceProxy(self.name, self._class)
        self._nodes = {}
        self.counter = 0
        self.expressions = {}

    def callService(self, *args):
        request = self._class._request_class()
        self.fill_message_slots(request, self.name, self.expression, self.counter)

        self.proxy.call(args)


def refresh_services(idx, servicesopc):
    global servicesDict

    services = rosservice.get_service_list()

    for service_name in services:
        try:
            if service_name not in servicesDict or servicesDict[service_name] is None:
                service = OpcUaROSService(servicesopc, idx, service_name,
                                          rosservice.get_service_class_by_name(service_name))
                servicesDict[service_name] = service
                servicesopc.add_method(idx, service_name, service.callService(), )
        except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
            server.stop()


def main(args):
    global server
    global servicesDict

    rospy.init_node("opcua_server")
    servicesDict = {}
    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:21554/")
    server.set_server_name("ROS ua Server")

    server.start()  # setup our own namespace, this is expected
    uri = "http://ros.org"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our custom stuff
    objects = server.get_objects_node()

    servicesopc = objects.add_object(idx, "ROS-Services")

    while True:
        refresh_services(idx, servicesopc)
        time.sleep(2)


if __name__ == "__main__":
    main(sys.argv)
