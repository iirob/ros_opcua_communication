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
    namespace_ros = args[1]
    print (namespace_ros)
    topicsDict = {}
    servicesDict = {}
    actionsDict = {}
    rospy.init_node("opcua_server")

    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:21554/")
    server.set_server_name("ROS ua Server")

    server.start()
    # setup our own namespaces, this is expected
    uri_topics = "http://ros.org/topics"
    # two different namespaces to make getting the correct node easier for get_node (otherwise had object for service and topic with same name
    uri_services = "http://ros.org/services"
    uri_actions = "http://ros.org/actions"
    idx_topics = server.register_namespace(uri_topics)
    idx_services = server.register_namespace(uri_services)
    idx_actions = server.register_namespace(uri_actions)
    # get Objects node, this is where we should put our custom stuff
    objects = server.get_objects_node()

    topics_object = objects.add_object(idx_topics, "ROS-Topics")
    services_object = objects.add_object(idx_services, "ROS-Services")
    actions_object = objects.add_object(idx_actions, "ROS_Actions")
    while not rospy.is_shutdown():
        ros_topics.refresh_topics_and_actions(namespace_ros, server, topicsDict, actionsDict, idx_topics, idx_actions, topics_object, actions_object)
        ros_services.refresh_services(namespace_ros, server, servicesDict, idx_services, services_object)
        # Don't clog cpu
        time.sleep(5)

    server.stop()
    quit()


if __name__ == "__main__":
    main(sys.argv)
