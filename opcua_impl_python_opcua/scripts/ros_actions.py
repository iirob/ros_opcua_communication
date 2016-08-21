# !/usr/bin/env python
# thanks to https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_action/src/rqt_action/action_plugin.py
import random
from pydoc import locate

import actionlib
import rospy
from opcua import ua, common
from opcua import uamethod

import ros_server
import ros_topics


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


class OpcUaROSAction:
    def __init__(self, server, parent, idx, name):
        self.server = server
        self.idx = idx
        self.name = name
        pkg = rospy.get_param("/rosopcua/actionpackage")
        msg_name = self.get_msg_name()
        class_name = msg_name.replace("_", "")
        goal_name = self.get_goal_name()
        try:
            actionspec = locate(pkg + "." + msg_name)
            goalspec = locate(pkg + "." + goal_name)
            print(goalspec)
            print(goal_name.replace("_", ""))
            self.goal_class = getattr(goalspec, goal_name.replace("_", ""))
            self.client = actionlib.SimpleActionClient(class_name.lower(), getattr(actionspec, class_name))
            self.parent = self.recursive_create_objects(name, idx, parent)

        except (ValueError, TypeError) as e:
            print(e)

        result = self.parent.add_object(ua.NodeId("result", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                        ua.QualifiedName("result", parent.nodeid.NamespaceIndex))
        self.result_node = ros_topics._create_node_with_type(result, self.idx, "goal_status", "goal_status", "string", -1)

        self.result_node.set_value(repr("No goal sent yet"))
        goal = self.parent.add_object(ua.NodeId("goal", self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                      ua.QualifiedName("goal", parent.nodeid.NamespaceIndex))
        goal_node = goal.add_method(idx, "send_goal", self.send_goal, [], [])

    def recursive_create_objects(self, name, idx, parent):
        hierachy = name.split('/')
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

    @uamethod
    def send_goal(self, parent, *inputs):
        print("sending goal")
        try:
            self.client.send_goal(self.goal_class())
        except Exception as e:
            print(e)
        self.result_node.set_value(repr(self.client.get_goal_status_text()))
        return


def refresh_dict(actionsdict, server, idx_actions):
    pass
