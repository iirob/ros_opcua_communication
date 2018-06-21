#!/usr/bin/python

from ros_global import *
from ros_messages import OpcUaROSMessage
from ros_services import OpcUaROSService
from ros_opc_ua import *

from datetime import datetime


class ROSServer(BasicROSServer):

    @staticmethod
    def get_ros_class(name):
        return getattr(ua, to_camel_case(name))

    def create_messages(self):
        rospy.loginfo(' ----- start creating messages ------ ')
        self.ros_data_types = OpcUaROSMessage(self.server, self.idx, self._idx_name).create_ros_data_types()
        # Generate python ua class according to ROSDictionary and register them
        self.server.load_type_definitions()
        rospy.loginfo(' ----- %s messages created------ ' % str(len(self.ros_data_types)))

    def create_service(self):
        OpcUaROSService(self.server, self.idx, self.ros_data_types)


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.create_messages()

            test_message = 'std_msgs/Header'
            test_dt = ua_server.get_ros_data_type_id(test_message)
            new_var = ua_server.server.nodes.objects.add_variable(nodeid_generator(ua_server.idx), 'testHeader',
                                                                  ua.Variant(None, ua.VariantType.Null),
                                                                  datatype=test_dt)
            new_var.set_writable()
            msg = ua_server.get_ros_class(test_message)()
            msg.frame_id = 'test frame id'
            msg.seq = 20
            msg.stamp = datetime.now()
            new_var.set_value(msg)
            result = new_var.get_value()

            test_message = 'std_msgs/Int32MultiArray'
            test_dt = ua_server.get_ros_data_type_id(test_message)
            array_test = ua_server.server.nodes.objects.add_variable(nodeid_generator(ua_server.idx), 'Intarray',
                                                                     ua.Variant(None, ua.VariantType.Null),
                                                                     datatype=test_dt)
            array_test.set_writable()
            array = ua_server.get_ros_class(test_message)()
            array.data = [1, 1, 2, 3, 5, 7, 9]
            array_test.set_value(array)

            result1 = array_test.get_value()
            # TODO: create nodes management to automatically do this.
            new_node = ua_server.server.nodes.objects.add_object(nodeid_generator(ua_server.idx), 'turtlesim')
            OpcUaROSService('/turtle1/teleport_relative', new_node, nodeid_generator(ua_server.idx),
                            ua_server.get_ros_data_type_id('turtlesim/TeleportRelativeRequest'),
                            ua_server.get_ros_data_type_id('turtlesim/TeleportRelativeResponse'))

            ua_server.start_server()
            rospy.spin()
    except Exception as e:
        print(e.message)
    # ua_server = ROSServer()
    # ua_server.create_messages()
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
