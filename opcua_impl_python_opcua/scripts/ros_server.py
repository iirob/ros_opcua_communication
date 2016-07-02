import sys
import time

import rospy
from opcua import Server

import ros_services
import ros_topics

global server


def shutdown():
    global server

    server.stop()


def main(args):
    global server

    topicsDict = {}
    servicesDict = {}
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

        topics = objects.add_object(idx, "ROS-Topics")
        servicesopc = objects.add_object(idx, "ROS-Services")
        while True:
            ros_topics.refresh_topics(server, topicsDict, idx, topics)
            ros_services.refresh_services(server, servicesDict, idx, servicesopc)
            # Don't clog cpu
            time.sleep(2)
        rospy.spin()

    except rospy.ROSInterruptException:

        server.stop()


if __name__ == "__main__":
    main(sys.argv)
