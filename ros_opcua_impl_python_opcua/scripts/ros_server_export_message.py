#!/usr/bin/python

from ros_global import *
from ros_opc_ua import to_camel_case, nodeid_generator
from ros_messages import OpcUaROSMessage
from ros_services import OpcUaROSService
from ros_topics import OpcUaROSTopicPub, OpcUaROSTopicSub


class ROSServer(BasicROSServer):

    @staticmethod
    def get_ros_class(name):
        return getattr(ua, to_camel_case(name))

    def load_messages(self):
        rospy.loginfo(' ----- start creating messages ------ ')
        self.ros_msgs = OpcUaROSMessage(self.server, self.idx, self._idx_name).create_ros_data_types()
        # Generate python ua class according to ROSDictionary and register them
        self.server.load_type_definitions()
        rospy.loginfo(' ----- %s messages created------ ' % str(len(self.ros_msgs)))

    def create_nodes(self):
        ros_nodes = get_nodes_info(self.ros_node_name)
        for node in ros_nodes.items():
            # Take rosout and self away from display
            if node[0] in ['/rosout', '/' + self.ros_node_name]:
                continue
            ua_node = ua_server.server.nodes.objects.add_object(nodeid_generator(self.idx), node[0])
            for service in node[1]['srvs']:
                OpcUaROSService(service, ua_node, nodeid_generator(self.idx), self.ros_msgs)
            for publish in node[1]['pubs']:
                if publish == '/rosout':  # Take rosout away
                    continue
                OpcUaROSTopicPub(publish, ua_node, nodeid_generator(self.idx), self.ros_msgs)
            for subscribe in node[1]['subs']:
                OpcUaROSTopicSub(subscribe, ua_node, nodeid_generator(self.idx),
                                 nodeid_generator(self.idx), self.ros_msgs)


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.load_messages()
            ua_server.create_nodes()
            # test_message = 'std_msgs/Header'
            # test_dt = ua_server.get_ros_data_type_id(test_message)
            # new_var = ua_server.server.nodes.objects.add_variable(nodeid_generator(ua_server.idx), 'testHeader',
            #                                                       ua.Variant(None, ua.VariantType.Null),
            #                                                       datatype=test_dt)
            # new_var.set_writable()
            # msg = ua_server.get_ros_class(test_message)()
            # msg.frame_id = 'test frame id'
            # msg.seq = 20
            # new_var.set_value(msg)
            # result = new_var.get_value()

            ua_server.start_server()
            rospy.spin()
    except Exception as e:
        print(str(e))
    # ua_server = ROSServer()
    # ua_server.load_messages()
    # ua_server.create_nodes()
    # test_message = 'std_msgs/Header'
    # test_dt = ua_server.get_ros_data_type_id(test_message)
    # new_var = ua_server.server.nodes.objects.add_variable(nodeid_generator(ua_server.idx), 'testHeader',
    #                                                       ua.Variant(None, ua.VariantType.Null),
    #                                                       datatype=test_dt)
    # new_var.set_writable()
    # msg = ua_server.get_ros_class(test_message)()
    # msg.frame_id = 'test frame id'
    # msg.seq = 20
    # msg.stamp = datetime.now()
    # new_var.set_value(msg)
    # result = new_var.get_value()
    #
    # new_node = ua_server.server.nodes.objects.add_object(nodeid_generator(ua_server.idx), 'turtlesim')
    # OpcUaROSService('/turtle1/teleport_relative', new_node, nodeid_generator(ua_server.idx),
    #                 ua_server.get_ros_data_type_id('turtlesim/TeleportRelativeRequest'),
    #                 ua_server.get_ros_data_type_id('turtlesim/TeleportRelativeResponse'))
    # ua_server.start_server()
    # rospy.init_node('rosopcua', log_level=rospy.INFO)
    # rospy.spin()
