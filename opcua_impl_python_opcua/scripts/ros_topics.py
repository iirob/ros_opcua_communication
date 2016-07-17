#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_topic/src/rqt_topic/topic_widget.py
import numpy
import random

import roslib
import roslib.message
import rospy
from opcua import ua, uamethod

import ros_server


class OpcUaROSTopic:
    def __init__(self, server, parent, idx, topic_name, topic_type):
        self.server = server
        self.parent = self.recursive_create_objects(topic_name, idx, parent)
        self.type_name = topic_type
        self.name = topic_name
        self._nodes = {}
        self.idx = idx

        self.message_class = None
        try:
            self.message_class = roslib.message.get_message_class(topic_type)
            self.message_instance = self.message_class()

        except Exception as e:
            self.message_class = None
            rospy.logfatal("There is not found message type class " + topic_type)

        self._recursive_create_items(self.parent, idx, topic_name, topic_type, self.message_instance, True)

        self._subscriber = rospy.Subscriber(self.name, self.message_class, self.message_callback)
        self._publisher = rospy.Publisher(self.name, self.message_class, queue_size=1)

    def _recursive_create_items(self, parent, idx, topic_name, type_name, message, top_level=False):
        hierachy = topic_name.split('/')
        topic_text = topic_name.split('/')[-1]
        # if hierachy[0] == topic_text:
        if '[' in topic_text:
            topic_text = topic_text[topic_text.index('['):]

        # This here are 'complex data'
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            complex_type = True
            new_node = parent.add_object(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                         ua.QualifiedName(topic_name, parent.nodeid.NamespaceIndex))
            new_node.add_property(ua.NodeId(topic_name + ".Type", idx),
                                  ua.QualifiedName("Type", parent.nodeid.NamespaceIndex), type_name)
            if top_level:
                new_node.add_method(ua.NodeId(topic_name + ".Update", parent.nodeid.NamespaceIndex),
                                    ua.QualifiedName("Update", parent.nodeid.NamespaceIndex),
                                    self.opcua_update_callback, [], [])
            for slot_name, type_name_child in zip(message.__slots__, message._slot_types):
                self._recursive_create_items(new_node, idx, topic_name + '/' + slot_name, type_name_child,
                                             getattr(message, slot_name))
            self._nodes[topic_name] = new_node

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

        if self._nodes[topic_name] is not None and self._nodes[topic_name].get_node_class() == ua.NodeClass.Variable:
            self._nodes[topic_name].set_writable(True)
        return

    def message_callback(self, message):
        self.update_value(self.name, message)

    @uamethod
    def opcua_update_callback(self, parent):
        try:
            for nodeName in self._nodes:
                child = self._nodes[nodeName]
                name = child.get_display_name().Text
                if hasattr(self.message_instance, name):
                    if child.get_node_class() == ua.NodeClass.Variable:
                        setattr(self.message_instance, name,
                                correct_type(child, type(getattr(self.message_instance, name))))
                    elif child.get_node_class == ua.NodeClass.Object:
                        setattr(self.message_instance, name, self.create_message_instance(child))
            self._publisher.publish(self.message_instance)
        except Exception as e:
            print(e)
            self.server.delete_nodes([self.parent])

    def update_value(self, topic_name, message):
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name in message.__slots__:
                self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name))

        elif type(message) in (list, tuple):
            if (len(message) > 0) and hasattr(message[0], '__slots__'):
                for index, slot in enumerate(message):
                    if topic_name + '[%d]' % index in self._nodes:
                        self.update_value(topic_name + '[%d]' % index, slot)
                    else:
                        base_type_str, _ = _extract_array_info(
                            self._nodes[topic_name].text(self.type_name))
                        self._recursive_create_items(self._nodes[topic_name], topic_name + '[%d]' % index,
                                                     base_type_str,
                                                     slot, None)
            # remove obsolete children
            if len(message) < len(self._nodes[topic_name].get_children()):
                for i in range(len(message), self._nodes[topic_name].childCount()):
                    item_topic_name = topic_name + '[%d]' % i
                    self.recursive_delete_items(self._nodes[item_topic_name])
                    del self._nodes[item_topic_name]
        else:
            if topic_name in self._nodes and self._nodes[topic_name] is not None:
                self._nodes[topic_name].set_value(repr(message))

    def recursive_delete_items(self, item):
        self._publisher.unregister()
        self._subscriber.unregister()
        for child in item.get_children():
            self.recursive_delete_items(child)
            if child in self._nodes:
                del self._nodes[child]
            self.server.delete_nodes([child])
        self.server.delete_nodes([item])

    def create_message_instance(self, node):
        for child in node.get_children():
            name = child.get_display_name().Text
            if hasattr(self.message_instance, name):
                if child.get_node_class() == ua.NodeClass.Variable:
                    setattr(self.message_instance, name,
                            correct_type(child, type(getattr(self.message_instance, name))))
                elif child.get_node_class == ua.NodeClass.Object:
                    setattr(self.message_instance, name, self.create_message_instance(child))
        return self.message_instance  # Converts the value of the node to that specified in the ros message we are trying to fill. Casts python ints

    def recursive_create_objects(self, topic_name, idx, parent):
        hierachy = topic_name.split('/')
        if len(hierachy) == 0 or len(hierachy) == 1:
            return parent
        for name in hierachy:
            if name != '':
                try:
                    nodewithsamename = self.server.get_node(ua.NodeId(name, idx))
                    if nodewithsamename is not None and nodewithsamename.get_parent() == parent:
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, nodewithsamename)
                    else:
                        newparent = parent.add_object(
                            ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, newparent)
                except Exception:
                    newparent = parent.add_object(
                        ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, newparent)

        return parent


# to unsigned integers as to fullfill ros specification. Currently only uses a few different types,
# no other types encountered so far.
def correct_type(node, typemessage):
    data_value = node.get_data_value()
    result = node.get_value()
    if isinstance(data_value, ua.DataValue):
        if typemessage.__name__ == "float":
            result = numpy.float(result)
        if typemessage.__name__ == "double":
            result = numpy.double(result)
        if typemessage.__name__ == "int":
            result = int(result) & 0xff
    else:
        print (node.get_data_value.Value)
        return None
    return result


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
    elif type_name == 'float' or type_name == 'float32' or type_name == 'float64':
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


def numberofsubscribers(nametolookfor, topicsDict):
    if nametolookfor != "/rosout":
        ret = topicsDict[nametolookfor]._subscriber.get_num_connections()
    else:
        ret = 2
    return ret


def refresh_topics(server, topicsdict, idx, topics):
    ros_topics = rospy.get_published_topics()

    for topic_name, topic_type in ros_topics:
        if topic_name not in topicsdict or topicsdict[topic_name] is None:
            topic = OpcUaROSTopic(server, topics, idx, topic_name, topic_type)
            topicsdict[topic_name] = topic

        elif numberofsubscribers(topic_name, topicsdict) <= 1:
            topicsdict[topic_name].recursive_delete_items(server.get_node(ua.NodeId(topic_name, idx)))
            del topicsdict[topic_name]

    ros_topics = rospy.get_published_topics()
    tobedeleted = []
    for topic_nameOPC in topicsdict:
        found = False
        for topicROS, topic_type in ros_topics:
            if topic_nameOPC == topicROS:
                found = True
        if not found:
            topicsdict[topic_nameOPC].recursive_delete_items(server.get_node(ua.NodeId(topic_nameOPC, idx)))
            tobedeleted.append(topic_nameOPC)
        if len(topicsdict[topic_nameOPC].parent.get_children()) <= 1 and "rosout" not in topic_nameOPC:
            print("deleting")
            print(topic_nameOPC)
            server.delete_nodes([topicsdict[topic_nameOPC].parent])
    for name in tobedeleted:
        print("deleting")
        print(name)
        del topicsdict[name]
