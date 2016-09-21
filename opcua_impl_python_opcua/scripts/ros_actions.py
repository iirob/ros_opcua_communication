# !/usr/bin/env python
# thanks to https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_action/src/rqt_action/action_plugin.py
import random
from pydoc import locate

import actionlib
import roslib
import rospy
from opcua import ua, common
from opcua import uamethod
from opcua.common.uaerrors import UaError
from roslib import message
from cob_sound.msg._SayAction import SayAction

import ros_server
import ros_services
import ros_topics


class OpcUaROSAction:
    def __init__(self, server, parent, idx, name, action_type, feedback_type):
        self.server = server
        self.idx = idx
        self.name = name
        self.type = action_type.split("/")[0]
        self.feedback_type = feedback_type
        self._feedback_nodes = {}
        goal_name = "_" + action_type.split("/")[-1]
        msg_name = goal_name.replace("Goal", "")
        class_name = msg_name.replace("_", "", 1)
        rospy.loginfo("Trying to find module with name: " + self.type + ".msg." + goal_name.replace("Goal", ""))
        actionspec = locate(self.type + ".msg." + msg_name )
        rospy.loginfo("We are creating action: " + self.name)
        rospy.loginfo("We have type: " + self.type)
        rospy.loginfo("We have msg name: " + msg_name)
        rospy.loginfo("We have class name: " + class_name)
        rospy.loginfo("We have goal name: " + goal_name)
        rospy.loginfo("We have goal class name: " + goal_name.replace("_", "", 1))

        goalspec = locate(self.type + ".msg." + goal_name)
        self.goal_class = getattr(goalspec, goal_name.replace("_", "", 1))
        try:
            self.client = actionlib.SimpleActionClient(self.get_ns_name(), getattr(actionspec, class_name))
            self.client.wait_for_server()
            rospy.loginfo("We have created a SimpleActionClient for action " + self.name)
        except actionlib.ActionException as e:
            rospy.logerr("Error when creating ActionClient for action " + self.name, e)
        self.parent = self.recursive_create_objects(name, idx, parent)
        self.result = self.parent.add_object(ua.NodeId(self.name + "_result", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                             ua.QualifiedName("result", parent.nodeid.NamespaceIndex))
        self.result_node = ros_topics._create_node_with_type(self.result, self.idx, self.name + "_result_value", self.name + "_result_value",
                                                             "string", -1)

        self.result_node.set_value("No goal completed yet")
        self.goal = self.parent.add_object(ua.NodeId(self.name + "_goal", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                           ua.QualifiedName("goal", parent.nodeid.NamespaceIndex))

        self.goal_node = self.goal.add_method(idx, self.name + "_send_goal", self.send_goal,
                                              create_arg_array(self.goal_class._slot_types, self.goal_class), [])

        self.goal_cancel = self.goal.add_method(idx, self.name + "_cancel_goal", self.cancel_goal, [], [])

        self.feedback = self.parent.add_object(ua.NodeId(self.name + "feedback", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                               ua.QualifiedName("feedback", parent.nodeid.NamespaceIndex))
        if self.feedback_type is not None:
            try:
                rospy.loginfo("We are trying to create Feedback for feedback type: " + self.feedback_type)
                self.feedback_message_class = roslib.message.get_message_class(self.feedback_type)
                self.feedback_message_instance = self.feedback_message_class()

            except rospy.ROSException:
                self.message_class = None
                rospy.logerror("Didn't find feedback message class for type " + self.feedback_type)

            self._recursive_create_feedback_items(self.feedback, self.name + "/feedback", self.feedback_type,
                                              getattr(self.feedback_message_instance, "feedback"))

        self.status = self.parent.add_object(ua.NodeId(self.name + "status", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                             ua.QualifiedName("status", parent.nodeid.NamespaceIndex))
        self.status_node = ros_topics._create_node_with_type(self.status, self.idx, self.name + "_status", self.name + "_status",
                                                             "string", -1)
        self.status_node.set_value("No goal sent yet")

    def message_callback(self, message):
        self.update_feedback_node(message)

    def update_feedback_node(self, feedback_message):
        for name in self._feedback_nodes:
            attr_name = name.split("/")[-1]
            try:
                current_attribute = getattr(feedback_message, attr_name)
                self._feedback_nodes[name].set_value(current_attribute)
            except (AttributeError, UaError) as e:
                rospy.logerr("Error occured when updating the feedback value %s", e)

    @uamethod
    def cancel_goal(self, parent, *inputs):
        rospy.loginfo("cancelling goal " + self.name)
        try:
            self.client.cancel_all_goals()
            self.update_state()
        except (rospy.ROSException, common.uaerrors.UaError) as e:
            rospy.logerr("Error when cancelling a goal for " + self.name, e)

    def recursive_create_objects(self, name, idx, parent):
        hierachy = name.split('/')
        if len(hierachy) == 0 or len(hierachy) == 1:
            return parent
        for name in hierachy:
            if name != '':
                try:
                    # This Happens if a topic exists with the exact same name so we have to workaround the fact an
                    # object with this parent name already exists, as we dont want our action nodes in the ros_topic namespace
                    nodewithsamename = self.server.get_node(ua.NodeId(name, idx))
                    if nodewithsamename is not None and nodewithsamename.get_parent() == parent:
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, nodewithsamename)
                    else:
                        newparent = parent.add_object(
                            ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, newparent)
                except IndexError, common.uaerrors.UaError:
                    newparent = parent.add_object(
                        ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx, newparent)
        return parent

    def _recursive_create_feedback_items(self, parent, feedback_topic_name, type_name, feedback_message):
        idx = self.idx
        topic_text = feedback_topic_name.split('/')[-1]
        if '[' in topic_text:
            topic_text = topic_text[topic_text.index('['):]

        # This here are 'complex data'
        if hasattr(feedback_message, '__slots__') and hasattr(feedback_message, '_slot_types'):
            complex_type = True
            new_node = parent.add_object(ua.NodeId(feedback_topic_name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                         ua.QualifiedName(feedback_topic_name, parent.nodeid.NamespaceIndex))

            for slot_name, type_name_child in zip(feedback_message.__slots__, feedback_message._slot_types):
                self._recursive_create_feedback_items(new_node, feedback_topic_name + '/' + slot_name, type_name_child,
                                                      getattr(feedback_message, slot_name))
            self._feedback_nodes[feedback_topic_name] = new_node

        else:
            # This are arrays
            base_type_str, array_size = ros_topics._extract_array_info(type_name)
            try:
                base_instance = roslib.message.get_message_class(base_type_str)()
            except (ValueError, TypeError):
                base_instance = None

            if array_size is not None and hasattr(base_instance, '__slots__'):
                for index in range(array_size):
                    self._recursive_create_feedback_items(parent, feedback_topic_name + '[%d]' % index, base_type_str, base_instance)
            else:
                new_node = ros_topics._create_node_with_type(parent, idx, feedback_topic_name, topic_text, type_name, array_size)
                new_node.set_writable(True)
                self._feedback_nodes[feedback_topic_name] = new_node
        return

    # namespace
    def get_ns_name(self):
        return str(self.name.split("/")[1])

    @uamethod
    def send_goal(self, parent, *inputs):
        rospy.loginfo("Sending Goal for " + self.name)
        try:
            self.client.send_goal(self.goal_class(*inputs), done_cb=self.update_result, feedback_cb=self.update_feedback, active_cb=self.update_state)
        except rospy.ROSException as e:
            rospy.logwarn(e)

    def recursive_delete_items(self, item):
        self.client.cancel_all_goals()
        for child in item.get_children():
            self.recursive_delete_items(child)
            self.server.delete_nodes([child])
        self.server.delete_nodes([self.result, self.result_node, self.goal_node, self.goal, self.parent])
        ros_server.own_rosnode_cleanup()

    def update_result(self, state, result):
        self.status_node.set_value(map_status_to_string(state))
        self.result_node.set_value(repr(result))

    def update_state(self):
        self.status_node.set_value(repr(self.client.get_goal_status_text()))

    def update_feedback(self, feedback):
        self.message_callback(feedback)


def get_correct_name(topic_name):
    splits = topic_name.split("/")
    counter = 0
    counter2 = 0
    result = ""
    while counter < len(splits):
        if splits[-1] == splits[counter] and not counter == 1:
            while counter2 <= counter - 1:
                if counter2 != counter - 1:
                    result += splits[counter2] + '/'
                else:
                    result += splits[counter2]
                counter2 += 1
            return result
        counter += 1


def create_arg_array(types, goal_class):
    array = []
    i = 0
    for actual_type in types:
        if "int" or "string" or "float" or "boolean" in actual_type:
            arg = ua.Argument()
            arg.Name = goal_class.__slots__[i]
            arg.DataType = ua.NodeId(ros_services.getobjectidfromtype(actual_type))
            arg.ValueRank = -1
            arg.ArrayDimensions = []
            arg.Description = ua.LocalizedText(arg.Name)
            array.append(arg)
        else:
            if "list" in actual_type:
                arg = ua.Argument()
                arg.Name = goal_class.__slots__[i]
                arg.DataType = ua.NodeId(ros_services.getobjectidfromtype("array"))
                arg.ValueRank = -1
                arg.ArrayDimensions = []
                arg.Description = ua.LocalizedText("Array")
            else:
                 array_to_merge = create_arg_array(actual_type)
                 rospy.loginfo("trying to create arg array for custom type: " + str(actual_type))
                 array.extend(array_to_merge)
        i += 1
    return array


def map_status_to_string(param):
    if param == 9:
        return "Goal LOST"
    elif param == 8:
        return "Goal RECALLED"
    elif param == 7:
        return "Goal RECALLING"
    elif param == 6:
        return "Goal PREEMPTING"
    elif param == 5:
        return "Goal REJECTED"
    elif param == 4:
        return "Goal ABORTED"
    elif param == 3:
        return "Goal SUCEEDED"
    elif param == 2:
        return "Goal PREEMPTED"
    elif param == 1:
        return "Goal ACTIVE"
    elif param == 0:
        return "Goal PENDING"


def refresh_dict(namespace_ros, actionsdict, topicsdict, server, idx_actions):
    topics = rospy.get_published_topics(namespace_ros)
    tobedeleted = []
    for actionNameOPC in actionsdict:
        found = False
        for topicROS, topic_type in topics:
            ros_server.own_rosnode_cleanup()
            if actionNameOPC in topicROS:
                found = True
        if not found:
            actionsdict[actionNameOPC].recursive_delete_items(actionsdict[actionNameOPC].parent)
            tobedeleted.append(actionNameOPC)
            ros_server.own_rosnode_cleanup()
    for name in tobedeleted:
        del actionsdict[name]
