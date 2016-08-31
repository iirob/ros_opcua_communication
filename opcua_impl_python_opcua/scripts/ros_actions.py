# !/usr/bin/env python
# thanks to https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_action/src/rqt_action/action_plugin.py
import random
from pydoc import locate

import actionlib
import rospy
from opcua import ua, common
from opcua import uamethod

import ros_server
import ros_services
import ros_topics


class OpcUaROSAction:
    def __init__(self, server, parent, idx, name, action_type):
        self.server = server
        self.idx = idx
        self.name = name
        self.type = action_type.split("/")[0] + ".msg"
        msg_name = self.get_msg_name()
        class_name = msg_name.replace("_", "")
        goal_name = self.get_goal_name()
        actionspec = locate(self.type + "." + msg_name)
        goalspec = locate(self.type + "." + goal_name)
        self.goal_class = getattr(goalspec, goal_name.replace("_", ""))
        self.client = actionlib.SimpleActionClient(self.get_ns_name(), getattr(actionspec, class_name))
        self.client.wait_for_server()
        self.parent = self.recursive_create_objects(name, idx, parent)
        self.result = self.parent.add_object(ua.NodeId(self.name + "_result", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                             ua.QualifiedName("result", parent.nodeid.NamespaceIndex))
        self.result_node = ros_topics._create_node_with_type(self.result, self.idx, self.name + "_result_value", self.name + "_result_value",
                                                             "string", -1)

        self.result_node.set_value("No goal sent yet")
        self.goal = self.parent.add_object(ua.NodeId(self.name + "_goal", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                           ua.QualifiedName("goal", parent.nodeid.NamespaceIndex))

        self.goal_node = self.goal.add_method(idx, self.name + "_send_goal", self.send_goal,
                                              create_arg_array(self.goal_class._slot_types, self.goal_class), [])

        self.goal_cancel = self.goal.add_method(idx, self.name + "_cancel_goal", self.cancel_goal, [], [])

        self.feedback = self.parent.add_object(ua.NodeId(self.name + "feedback", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                               ua.QualifiedName("feedback", parent.nodeid.NamespaceIndex))
        self.feedback_node = ros_topics._create_node_with_type(self.feedback, self.idx, self.name + "_feedback", self.name + "_feedback",
                                                               "string", -1)
        self.feedback_node.set_value("No goal sent yet")
        self.status = self.parent.add_object(ua.NodeId(self.name + "status", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                             ua.QualifiedName("status", parent.nodeid.NamespaceIndex))
        self.status_node = ros_topics._create_node_with_type(self.status, self.idx, self.name + "_status", self.name + "_status",
                                                             "string", -1)
        self.status_node.set_value("No goal sent yet")

    @uamethod
    def cancel_goal(self, parent, *inputs):
        rospy.loginfo("cancelling goal " + self.name)
        try:
            self.client.cancel_all_goals()
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

    def get_msg_name(self):
        return "_" + str(self.name.split("/")[-1]).capitalize() + "Action"

    def get_goal_name(self):
        return "_" + str(self.name.split("/")[-1]).capitalize() + "Goal"

    # namespace
    def get_ns_name(self):
        return str(self.name.split("/")[1])

    @uamethod
    def send_goal(self, parent, *inputs):
        rospy.loginfo("Sending Goal for " + self.name)
        try:
            self.client.send_goal(self.goal_class(*inputs))
        except rospy.ROSException as e:
            rospy.logwarn(e)

    def recursive_delete_items(self, item):
        self.client.cancel_all_goals()
        for child in item.get_children():
            self.recursive_delete_items(child)
            self.server.delete_nodes([child])
        self.server.delete_nodes([self.result, self.result_node, self.goal_node, self.goal, self.parent])
        ros_server.own_rosnode_cleanup()


def present_in_actions_dict(actionsdict, name):
    for opc_name in actionsdict:
        if opc_name == name:
            return True

    return False


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
                arg = ua.Argument()
                arg.Name = goal_class.__slots__[i]
                arg.DataType = ua.NodeId(ros_services.getobjectidfromtype(actual_type))
                arg.ValueRank = -1
                arg.ArrayDimensions = []
                arg.Description = ua.LocalizedText("Object")
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
                try:
                    actionsdict[actionNameOPC].result_node.set_value(repr(actionsdict[actionNameOPC].client.get_result()))
                    actionsdict[actionNameOPC].feedback_node.set_value(map_status_to_string((actionsdict[actionNameOPC].client.get_state())))
                    actionsdict[actionNameOPC].status_node.set_value(repr(actionsdict[actionNameOPC].client.get_goal_status_text()))
                except (rospy.ROSException, AttributeError) as e:
                    rospy.loginfo("No goal sent yet or error in creation of an Action")
                    try:
                        rospy.loginfo(e)
                    except TypeError as e2:
                        rospy.loginfo("Error when Logging Exception", e2)
        if not found:
            actionsdict[actionNameOPC].recursive_delete_items(actionsdict[actionNameOPC].parent)
            tobedeleted.append(actionNameOPC)
            ros_server.own_rosnode_cleanup()
    for name in tobedeleted:
        del actionsdict[name]
