import sys
import time

import rospy
from opcua import Server

import ros_services
import ros_topics


# Returns the hierachy as one string from the first remaining part on.
def nextname(hierachy, index_of_last_processed):
    output = ""
    counter = index_of_last_processed + 1
    while counter < len(hierachy):
        output += hierachy[counter]
        counter += 1
    return output


def main(args):
    topicsDict = {}
    servicesDict = {}
    rospy.init_node("opcua_server")

    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:21554/")
    server.set_server_name("ROS ua Server")

    server.start()  # setup our own namespace, this is expected
    uri_topics = "http://ros.org/topics"
    uri_services = "http://ros.org/services"
    idx_topics = server.register_namespace(uri_topics)
    idx_services = server.register_namespace(uri_services)
    # get Objects node, this is where we should put our custom stuff
    objects = server.get_objects_node()

    topics_object = objects.add_object(idx_topics, "ROS-Topics")
    services_object = objects.add_object(idx_services, "ROS-Services")
    while not rospy.is_shutdown():
        ros_topics.refresh_topics(server, topicsDict, idx_topics, topics_object)
        ros_services.refresh_services(server, servicesDict, idx_services, services_object)
        # Don't clog cpu
        time.sleep(5)

    server.stop()
    quit()


if __name__ == "__main__":
    main(sys.argv)
