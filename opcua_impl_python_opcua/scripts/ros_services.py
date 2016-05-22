#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import sys

import rospy
import rosservice
from opcua import Server

global server


class OpcUaROSService:
    global server


def main(args):
    global server

    rospy.init_node("opcua_server")

    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:21554/")
    server.set_server_name("ROS ua Server")

    server.start()

    try:
        # setup our own namespace, this is expected
        uri = "http://ros.org"
        idx = server.register_namespace(uri)

        # get Objects node, this is where we should put our custom stuff
        objects = server.get_objects_node()

        objects.add_object(idx, "ROS-Services")

        services = rosservice.get_service_list()

        for service_name in services
            try:
                service = OpcUaROSService(rosservice.get_service_class_by_name(service_name))
                self._services[service_name] = rosservice.get_service_class_by_name(service_name)
                # qDebug('ServiceCaller.on_refresh_services_button_clicked(): found service %s using class %s' % (service_name, self._services[service_name]))
            except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
                server.stop()
        rospy.spin()

    except rospy.ROSInterruptException:

        server.stop()


if __name__ == "__main__":
    main(sys.argv)
