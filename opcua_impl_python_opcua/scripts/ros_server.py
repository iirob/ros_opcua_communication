#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_topic/src/rqt_topic/topic_widget.py
import sys
from PyQt4 import Qt

import roslib
import roslib.message
import rospy
from opcua import ua, Server, uamethod

global server


class OpcUaROSTopic:
    global server

    def __init__(self, parent, idx, topic_name, topic_type):

        self.type_name = topic_type
        self.name = topic_name

        self._nodes = {}

        self.message_class = None
        try:
            self.message_class = roslib.message.get_message_class(topic_type)
            self.message_instance = self.message_class()
        except Exception as e:
            self.message_class = None
            rospy.logfatal("There is not found message type class " + topic_type)

        self._recursive_create_items(parent, idx, topic_name, topic_type, self.message_instance, True)

        self._subscriber = rospy.Subscriber(self.name, self.message_class, self.message_callback)
        self._publisher = rospy.Publisher(self.name, self.message_class, queue_size=1)

    def _recursive_create_items(self, parent, idx, topic_name, type_name, message, top_level=False):
        topic_text = topic_name.split('/')[-1]
        if '[' in topic_text:
            topic_text = topic_text[topic_text.index('['):]

        # This here are 'complex data'
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            complex_type = True
            new_node = parent.add_object(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                         ua.QualifiedName(topic_text, parent.nodeid.NamespaceIndex))
            new_node.add_property(ua.NodeId(topic_name + ".Type", idx),
                                  ua.QualifiedName("Type", parent.nodeid.NamespaceIndex), type_name)
            if top_level:
                new_node.add_method(ua.NodeId(topic_name + ".Update", parent.nodeid.NamespaceIndex),
                                    ua.QualifiedName("Update", parent.nodeid.NamespaceIndex),
                                    self.opcua_update_callback, [], [])
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_items(new_node, idx, topic_name + '/' + slot_name, type_name,
                                             getattr(message, slot_name))

        else:
            # This are arrays
            base_type_str, array_size = _extract_array_info(type_name)
            try:
                base_instance = roslib.message.get_message_class(base_type_str)()
            except (ValueError, TypeError):
                base_instance = None

            if array_size is not None and hasattr(base_instance, '__slots__'):
                for index in range(array_size):
                    self._recursive_create_items(parent, idx, topic_name + '[%d]' % index, base_type_str, base_instance)
            else:
                new_node = _create_node_with_type(parent, idx, topic_name, topic_text, type_name, array_size)
        self._nodes[topic_name] = new_node
        if self._nodes[topic_name].get_node_class() == ua.NodeClass.Variable:
            self._nodes[topic_name].set_writable(True)
        return

    def message_callback(self, message):
        self.update_value(self.name, message)

    @uamethod
    def opcua_update_callback(self, parent):
        self._publisher.publish(self.message_instance)
        print self._nodes

    def update_value(self, topic_name, message):
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name in message.__slots__:
                self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name))

        elif type(message) in (list, tuple):
            if (len(message) > 0) and hasattr(message[0], '__slots__'):
                print("message received")
                for index, slot in enumerate(message):
                    if topic_name + '[%d]' % index in self._nodes:
                        self.update_value(topic_name + '[%d]' % index, slot)
                    else:
                        base_type_str, _ = _extract_array_info(
                            self._nodes[topic_name].text(self._column_index['type']))
                        self._recursive_create_items(self._nodes[topic_name], topic_name + '[%d]' % index,
                                                     base_type_str,
                                                     slot, None)
            # remove obsolete children
            if len(message) < len(self._nodes[topic_name].get_children()):
                for i in range(len(message), self._nodes[topic_name].childCount()):
                    item_topic_name = topic_name + '[%d]' % i
                    self._recursive_delete_items(self._nodes[item_topic_name])
            else:
                topic_name = repr(message).replace(":*", "", 1) + topic_name
                print(topic_name)
                if topic_name in self._nodes and self._nodes[topic_name] is not None:
                    self._nodes[topic_name].set_value(repr(message))


def _recursive_delete_widget_items(self, item):
    def _recursive_remove_items_from_tree(itemintree):
        for index in reversed(range(itemintree.childCount())):
            _recursive_remove_items_from_tree(itemintree.child(index))
        topic_name = itemintree.data(0, Qt.UserRole)
        del self._tree_items[topic_name]

    _recursive_remove_items_from_tree(item)
    item.parent().removeChild(item)


def _extract_array_info(type_str):
    array_size = None
    if '[' in type_str and type_str[-1] == ']':
        type_str, array_size_str = type_str.split('[', 1)
        array_size_str = array_size_str[:-1]
        if len(array_size_str) > 0:
            array_size = int(array_size_str)
        else:
            array_size = 0

    return type_str, array_size


def _create_node_with_type(parent, idx, topic_name, topic_text, type_name, array_size):
    if '[' in type_name:
        type_name = type_name[:type_name.index('[')]

    if type_name == 'bool':
        dv = ua.Variant(False, ua.VariantType.Boolean)
    elif type_name == 'byte':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int8':
        dv = ua.Variant(0, ua.VariantType.SByte)
    elif type_name == 'uint8':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int16':
        dv = ua.Variant(0, ua.VariantType.Int16)
    elif type_name == 'uint16':
        dv = ua.Variant(0, ua.VariantType.UInt16)
    elif type_name == 'int32':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif type_name == 'uint32':
        dv = ua.Variant(0, ua.VariantType.UInt32)
    elif type_name == 'int64':
        dv = ua.Variant(0, ua.VariantType.Int64)
    elif type_name == 'uint64':
        dv = ua.Variant(0, ua.VariantType.UInt64)
    elif type_name == 'float':
        dv = ua.Variant(0.0, ua.VariantType.Float)
    elif type_name == 'double':
        dv = ua.Variant(0.0, ua.VariantType.Double)
    elif type_name == 'string':
        dv = ua.Variant('', ua.VariantType.String)
    else:
        print (type_name)
        return None

    if array_size is not None:
        value = []
        for i in range(array_size):
            value.append(i)
    return parent.add_variable(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex),
                               ua.QualifiedName(topic_text, parent.nodeid.NamespaceIndex), dv.Value)


def shutdown():
    global server

    server.stop()


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
        # Add a variable

        myvar = objects.add_variable(idx, "myvar", 10)
        myvar.set_writable()

        # TODO: Add refresh method (namespace can be sent as paramter)

        topics = objects.add_object(idx, "ROS-Topics")
        # TODO: Add reading of system state

        ros_topics = rospy.get_published_topics()

        for topic_name, topic_type in ros_topics:
            topic = OpcUaROSTopic(topics, idx, topic_name, topic_type)
            print(topic.name)
        rospy.spin()

    except rospy.ROSInterruptException:

        server.stop()


if __name__ == "__main__":
    main(sys.argv)
