#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_topic/src/rqt_topic/topic_widget.py
import numpy
import random

import roslib
import roslib.message
import rospy
from opcua import ua, uamethod

import ros_actions
import ros_server
import rostopic
import ros_global
import ros_messages




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

        except rospy.ROSException:
            self.message_class = None
            rospy.logfatal("Couldn't find message class for type " + topic_type)

        self._recursive_create_items(self.parent, idx, topic_name, topic_type, self.message_instance, True)

        self._subscriber = rospy.Subscriber(self.name, self.message_class, self.message_callback)
        self._publisher = rospy.Publisher(self.name, self.message_class, queue_size=1)
        rospy.loginfo("Created ROS Topic with name: " + str(self.name))



    def _recursive_create_items(self, parent, idx, topic_name, type_name, message, top_level=False):

        topic_text = topic_name.split('/')[-1]
        if '[' in topic_text:
            topic_text = topic_text[topic_text.index('['):]


        #get variable typpe with the topic type
        variable_typ_node = ros_global.messageNode[type_name]

        #create a new instance of  variabletype and  add to name space or topics node in namespace
        new_variable_node = ros_messages.instantiate_custumizate(parent,
                                         variable_typ_node,
                                         dname = ua.LocalizedText(topic_name),
                                         idx=self.idx)
        #save  the new instance in  topic array
        ros_global.topicNode[topic_name] = new_variable_node
        self._nodes[topic_name] = new_variable_node
        #self._nodes[topic_name].set_writable(True)

        # add method to update
        parent.add_method (idx,  "update", self.opcua_update_callback)

        # subscribe for editing
        """
        print "parent"
        print parent.get_browse_name()
        eventNotifiers = parent.get_event_notifier()
        eventNotifiers.add(ua.EventNotifier.SubscribeToEvents)
        parent.set_event_notifier(eventNotifiers)
        handle = self.server.subscription.subscribe_events(parent, ua.ObjectIds.SemanticChangeEventType)
        if len(parent.get_variables()) > 0:
            parent.get_variables()[0].set_value("Mein Name")
        print " Hanlde  : "
        print handle
        """

    def message_callback(self, message):
        self.update_value(self.name, message)


    @uamethod
    def opcua_update_callback(self, parent):
        try:
            node    = ros_global.topicNode[self.name] #self._nodes.values()[0]
            message = ros_messages.update_message_instance_with_node(self.message_instance, node)
            self.message_instance = message
            """"
            for nodeName in self._nodes:
                child = self._nodes[nodeName]
                name = child.get_display_name().Text
                if hasattr(self.message_instance, name):
                    if child.get_node_class() == ua.NodeClass.Variable:
                        setattr(self.message_instance, name,
                                correct_type(child, type(getattr(self.message_instance, name))))
                    elif child.get_node_class == ua.NodeClass.Object:
                        setattr(self.message_instance, name, self.create_message_instance(child))
            """
            self._publisher.publish(self.message_instance)


        except rospy.ROSException as e:
            rospy.logerr("Error when updating node " + self.name, e)
          #  self.server.server.delete_nodes([self.parent])


    def update_value(self, topic_name, message):
        # check if exits
        # get topic node
        if topic_name in ros_global.topicNode.keys() :
            topic_node = ros_global.topicNode[topic_name]
            ros_messages.update_node_with_message(topic_node, message, self.idx)
            self.message_instance = message
        else :
            print "No found"

    def recursive_delete_items(self, item):
        self._publisher.unregister()
        self._subscriber.unregister()
        for child in item.get_children():
            self.recursive_delete_items(child)
            if child in self._nodes:
                del self._nodes[child]
            self.server.server.delete_nodes([child])
        self.server.server.delete_nodes([item])
        if len(self.parent.get_children()) == 0:
            self.server.server.delete_nodes([self.parent])

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
                    nodewithsamename = self.server.find_topics_node_with_same_name(name, idx)
                    if nodewithsamename is not None:
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                             nodewithsamename)
                    else:
                        # if for some reason 2 services with exactly same name are created use hack>: add random int, prob to hit two
                        # same ints 1/10000, should be sufficient
                        newparent = parent.add_object(
                            ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))

                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,newparent)
                # thrown when node with parent name is not existent in server
                except IndexError, common.UaError:
                    newparent = parent.add_object(
                        ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                                  ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                         newparent)
        return parent


# to unsigned integers as to fulfill ros specification. Currently only uses a few different types,
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
        rospy.logerr("can't convert: " + str(node.get_data_value.Value))
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
    elif type_name == 'int':
        dv = ua.Variant(0, ua.VariantType.Int32)
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
        rospy.logerr("can't create node with type" + str(type_name))
        return None

    if array_size is not None:
        value = []
        for i in range(array_size):
            value.append(i)
    return parent.add_variable(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex),
                               ua.QualifiedName(topic_text, parent.nodeid.NamespaceIndex), dv.Value)


# Used to delete obsolete topics
def numberofsubscribers(nametolookfor, topicsDict):
    # rosout only has one subscriber/publisher at all times, so ignore.
    if nametolookfor != "/rosout":
        ret = topicsDict[nametolookfor]._subscriber.get_num_connections()
    else:
        ret = 2
    return ret


def refresh_topics_and_actions(namespace_ros, server, topicsdict, actionsdict, idx_topics, idx_actions, topics,
                               actions):
    ros_topics = rospy.get_published_topics(namespace_ros)
    rospy.logdebug(str(ros_topics))
    rospy.logdebug(str(rospy.get_published_topics('/move_base_simple')))
    for topic_name, topic_type in ros_topics:
        if topic_name not in topicsdict or topicsdict[topic_name] is None:
            if "cancel" in topic_name or "result" in topic_name or "feedback" in topic_name or "goal" in topic_name or "status" in topic_name:
                rospy.logdebug("Found an action: " + str(topic_name))
                correct_name = ros_actions.get_correct_name(topic_name)
                if correct_name not in actionsdict:
                    rospy.loginfo("Creating Action with name: " + correct_name)
                    try:
                        actionsdict[correct_name] = ros_actions.OpcUaROSAction(server, actions, idx_actions,
                                                                               correct_name,
                                                                               get_goal_type(correct_name),
                                                                               get_feedback_type(
                                                                                   correct_name))
                    except (ValueError, TypeError, AttributeError) as e:
                        print(e)
                        rospy.logerr("Error while creating Action Objects for Action " + topic_name)

            else:
                # rospy.loginfo("Ignoring normal topics for debugging...")
                topic = OpcUaROSTopic(server, topics, idx_topics, topic_name, topic_type)
                topicsdict[topic_name] = topic
        elif numberofsubscribers(topic_name, topicsdict) <= 1 and "rosout" not in topic_name:
            topicsdict[topic_name].recursive_delete_items(server.server.get_node(ua.NodeId(topic_name, idx_topics)))
            del topicsdict[topic_name]
            ros_server.own_rosnode_cleanup()

    ros_topics = rospy.get_published_topics(namespace_ros)
    # use to not get dict changed during iteration errors
    tobedeleted = []
    for topic_nameOPC in topicsdict:
        found = False
        for topicROS, topic_type in ros_topics:
            if topic_nameOPC == topicROS:
                found = True
        if not found:
            topicsdict[topic_nameOPC].recursive_delete_items(server.get_node(ua.NodeId(topic_nameOPC, idx_topics)))
            tobedeleted.append(topic_nameOPC)
    for name in tobedeleted:
        del topicsdict[name]
    ros_actions.refresh_dict(namespace_ros, actionsdict, topicsdict, server, idx_actions)


def get_feedback_type(action_name):
    try:
        type, name, fn = rostopic.get_topic_type(action_name + "/feedback")
        return type
    except rospy.ROSException as e:
        try:
            type, name, fn = rostopic.get_topic_type(action_name + "/Feedback", e)
            return type
        except rospy.ROSException as e2:
            rospy.logerr("Couldnt find feedback type for action " + action_name, e2)
            return None


def get_goal_type(action_name):
    try:
        type, name, fn = rostopic.get_topic_type(action_name + "/goal")
        return type
    except rospy.ROSException as e:
        try:
            type, name, fn = rostopic.get_topic_type(action_name + "/Goal", e)
            return type
        except rospy.ROSException as e2:
            rospy.logerr("Couldnt find feedback type for action " + action_name, e2)
            return None
