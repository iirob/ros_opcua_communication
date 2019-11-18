# !/usr/bin/python
# thanks to https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_action/src/rqt_action/action_plugin.py
import string
import random
from pydoc import locate

import actionlib
import roslib
import rospy
from opcua import ua, common
from opcua import uamethod
from opcua.ua.uaerrors import UaError
from roslib import message

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
        rospy.logdebug("Trying to find module with name: " + self.type + ".msg." + goal_name.replace("Goal", ""))
        actionspec = locate(self.type + ".msg." + msg_name)
        rospy.logdebug("We are creating action: " + self.name)
        rospy.logdebug("We have type: " + self.type)
        rospy.logdebug("We have msg name: " + msg_name)
        rospy.logdebug("We have class name: " + class_name)
        rospy.logdebug("We have goal name: " + goal_name)
        rospy.logdebug("We have goal class name: " + goal_name.replace("_", "", 1))

        goalspec = locate(self.type + ".msg." + goal_name)
        rospy.logdebug("found goalspec")
        self.goal_class = getattr(goalspec, goal_name.replace("_", "", 1))
        # malformed move_base_simple Action hack
        if 'move_base_simple' in self.name:
            self.goal_instance = self.goal_class()
        else:
            self.goal_instance = self.goal_class().goal
        rospy.logdebug("found goal_instance " + str(self.goal_instance))
        try:
            self.client = actionlib.SimpleActionClient(self.get_ns_name(), getattr(actionspec, class_name))
            rospy.logdebug("We have created a SimpleActionClient for action " + self.name)
        except actionlib.ActionException as e:
            rospy.logerr("Error when creating ActionClient for action " + self.name, e)
        rospy.logdebug("Creating parent objects for action " + str(self.name))
        self.parent = self.recursive_create_objects(name, idx, parent)
        rospy.logdebug("Found parent for action: " + str(self.parent))
        rospy.logdebug("Creating main node with name " + self.name.split("/")[-1])
        # parent is our main node, this means our parent in log message above was actionsObject
        if self.name.split("/")[-1] == self.parent.nodeid.Identifier:
            self.main_node = self.parent
        else:
            self.main_node = self.parent.add_object(
                ua.NodeId(self.name.split("/")[-1], self.parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                ua.QualifiedName(self.name.split("/")[-1], self.parent.nodeid.NamespaceIndex))
        rospy.logdebug("Created Main Node under parent!")
        self.result = self.main_node.add_object(
            ua.NodeId(self.name + "_result", self.main_node.nodeid.NamespaceIndex, ua.NodeIdType.String),
            ua.QualifiedName("result", self.main_node.nodeid.NamespaceIndex))
        self.result_node = ros_topics._create_node_with_type(self.result, self.idx, self.name + "_result_value",
                                                             self.name + "_result_value",
                                                             "string", -1)

        self.result_node.set_value("No goal completed yet")
        rospy.logdebug("Created result node")
        self.goal = self.main_node.add_object(
            ua.NodeId(self.name + "_goal", self.main_node.nodeid.NamespaceIndex, ua.NodeIdType.String),
            ua.QualifiedName("goal", parent.nodeid.NamespaceIndex))
        self.goal_node = self.goal.add_method(idx, self.name + "_send_goal", self.send_goal,
                                              getargarray(self.goal_instance), [])

        self.goal_cancel = self.goal.add_method(idx, self.name + "_cancel_goal", self.cancel_goal, [], [])

        rospy.logdebug("Created goal node")

        self.feedback = self.main_node.add_object(
            ua.NodeId(self.name + "feedback", self.main_node.nodeid.NamespaceIndex, ua.NodeIdType.String),
            ua.QualifiedName("feedback", self.main_node.nodeid.NamespaceIndex))
        rospy.logdebug("Created feedback node")
        if self.feedback_type is not None:
            try:
                rospy.logdebug("We are trying to create Feedback for feedback type: " + self.feedback_type)
                self.feedback_message_class = roslib.message.get_message_class(self.feedback_type)
                self.feedback_message_instance = self.feedback_message_class()
                rospy.logdebug("Created feedback message instance")

            except rospy.ROSException:
                self.message_class = None
                rospy.logerror("Didn't find feedback message class for type " + self.feedback_type)

            self._recursive_create_feedback_items(self.feedback, self.name + "/feedback", self.feedback_type,
                                                  getattr(self.feedback_message_instance, "feedback"))

        self.status = self.main_node.add_object(
            ua.NodeId(self.name + "status", self.main_node.nodeid.NamespaceIndex, ua.NodeIdType.String),
            ua.QualifiedName("status", self.main_node.nodeid.NamespaceIndex))
        self.status_node = ros_topics._create_node_with_type(self.status, self.idx, self.name + "_status",
                                                             self.name + "_status",
                                                             "string", -1)
        self.status_node.set_value("No goal sent yet")
        rospy.loginfo("Created ROS Action with name: %s", self.name)

    def message_callback(self, message):
        self.update_value(self.name + "/feedback", message)

    def update_value(self, topic_name, message):
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name in message.__slots__:
                self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name))

        elif type(message) in (list, tuple):
            if (len(message) > 0) and hasattr(message[0], '__slots__'):
                for index, slot in enumerate(message):
                    if topic_name + '[%d]' % index in self._feedback_nodes:
                        self.update_value(topic_name + '[%d]' % index, slot)
                    else:
                        if topic_name in self._feedback_nodes:
                            base_type_str, _ = ros_topics._extract_array_info(
                                self._feedback_nodes[topic_name].text(self.feedback_type))
                            self._recursive_create_items(self._feedback_nodes[topic_name], topic_name + '[%d]' % index,
                                                         base_type_str,
                                                         slot, None)
            # remove obsolete children
            if topic_name in self._feedback_nodes:
                if len(message) < len(self._feedback_nodes[topic_name].get_children()):
                    for i in range(len(message), self._feedback_nodes[topic_name].childCount()):
                        item_topic_name = topic_name + '[%d]' % i
                        self.recursive_delete_items(self._feedback_nodes[item_topic_name])
                        del self._feedback_nodes[item_topic_name]
        else:
            if topic_name in self._feedback_nodes and self._feedback_nodes[topic_name] is not None:
                self._feedback_nodes[topic_name].set_value(repr(message))

    @uamethod
    def cancel_goal(self, parent, *inputs):
        # rospy.logdebug("cancelling goal " + self.name)
        try:
            self.client.cancel_all_goals()
            self.update_state()
        except (rospy.ROSException, UaError) as e:
            rospy.logerr("Error when cancelling a goal for " + self.name, e)

    def recursive_create_objects(self, topic_name, idx, parent):
        rospy.logdebug("reached parent object creation! current parent: " + str(parent))
        hierachy = topic_name.split('/')
        rospy.logdebug("Current hierachy: " + str(hierachy))
        if len(hierachy) == 0 or len(hierachy) == 1:
            return parent
        for name in hierachy:
            rospy.logdebug("current name: " + str(name))
            if name != '':
                try:
                    nodewithsamename = self.server.find_action_node_with_same_name(name, idx)
                    if nodewithsamename is not None:
                        rospy.logdebug("Found node with same name, is now new parent")
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                             nodewithsamename)
                    else:
                        # if for some reason 2 services with exactly same name are created use hack>: add random int, prob to hit two
                        # same ints 1/10000, should be sufficient
                        newparent = parent.add_object(
                            ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                             newparent)
                # thrown when node with parent name is not existent in server
                except IndexError, UaError:
                    newparent = parent.add_object(
                        ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                                  ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                         newparent)

        return parent

    def _recursive_create_feedback_items(self, parent, feedback_topic_name, type_name, feedback_message):
        idx = self.idx
        topic_text = feedback_topic_name.split('/')[-1]
        if '[' in topic_text:
            topic_text = topic_text[topic_text.index('['):]

        # This here are 'complex data'
        if hasattr(feedback_message, '__slots__') and hasattr(feedback_message, '_slot_types'):
            complex_type = True
            new_node = parent.add_object(
                ua.NodeId(feedback_topic_name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
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
                    self._recursive_create_feedback_items(parent, feedback_topic_name + '[%d]' % index, base_type_str,
                                                          base_instance)
            else:
                new_node = ros_topics._create_node_with_type(parent, idx, feedback_topic_name, topic_text, type_name,
                                                             array_size)
                new_node.set_writable(True)
                self._feedback_nodes[feedback_topic_name] = new_node
        return

    # namespace
    def get_ns_name(self):
        splits = self.name.split("/")
        ns = splits[1:]
        res = ""
        for split in ns:
            res += split + "/"
        rospy.logdebug("Created ns name: " + res[:-1])
        return str(res[:-1])

    @uamethod
    def send_goal(self, parent, *inputs):
        rospy.loginfo("Sending Goal for " + self.name)
        try:
            goal_msg = self.create_message_instance(inputs, self.goal_instance)
            if 'move_base' in self.name:
                rospy.loginfo("setting frame_id for move_base malformation")
                try:
                    target_pose = getattr(goal_msg, "target_pose")
                    header = getattr(target_pose, "header")
                    setattr(header, "frame_id", "map")
                    setattr(target_pose, "header", header)
                except AttributeError as e:
                    rospy.logerr("Error occured when setting frame_id", e)
            rospy.loginfo("Created Message Instance for goal-send: " + str(goal_msg))
            self.client.send_goal(goal_msg, done_cb=self.update_result, feedback_cb=self.update_feedback,
                                  active_cb=self.update_state)
            return
        except Exception as e:
            rospy.logerr("Error occured during goal sending for Action " + str(self.name))
            print(e)

    def create_message_instance(self, inputs, sample):
        rospy.logdebug("Creating message for goal call")
        already_set = []
        if isinstance(inputs, tuple):
            arg_counter = 0
            object_counter = 0
            while arg_counter < len(inputs) and object_counter < len(sample.__slots__):
                cur_arg = inputs[arg_counter]
                cur_slot = sample.__slots__[object_counter]
                # ignore header for malformed move_base_goal, as header shouldnt be in sent message
                while cur_slot == 'header':
                    rospy.logdebug("ignoring header")
                    object_counter += 1
                    if object_counter < len(sample.__slots__):
                        cur_slot = sample.__slots__[object_counter]
                real_slot = getattr(sample, cur_slot)
                rospy.lodebug(
                    "cur_arg: " + str(cur_arg) + " cur_slot_name: " + str(cur_slot) + " real slot content: " + str(
                        real_slot))
                if hasattr(real_slot, '_type'):
                    rospy.logdebug("We found an object with name " + str(cur_slot) + ", creating it recursively")
                    arg_counter_before = arg_counter
                    already_set, arg_counter = self.create_object_instance(already_set, real_slot, cur_slot,
                                                                           arg_counter, inputs, sample)
                    if arg_counter != arg_counter_before:
                        object_counter += 1
                    rospy.logdebug("completed object, object counter: " + str(object_counter) + " len(object): " + str(
                        len(sample.__slots__)))
                else:
                    already_set.append(cur_slot)
                    # set the attribute in the request
                    setattr(sample, cur_slot, cur_arg)
                    arg_counter += 1
                    object_counter += 1

        return sample

    def create_object_instance(self, already_set, object, name, counter, inputs, parent):
        rospy.logdebug("Create Object Instance Notify")
        object_counter = 0
        while object_counter < len(object.__slots__) and counter < len(inputs):
            cur_arg = inputs[counter]
            cur_slot = object.__slots__[object_counter]
            # ignore header for malformed move_base_goal, as header shouldnt be in sent message
            while cur_slot == 'header':
                rospy.logdebug("ignoring header")
                object_counter += 1
                if object_counter < len(object.__slots__):
                    cur_slot = object.__slots__[object_counter]
                else:
                    return already_set, counter
            real_slot = getattr(object, cur_slot)
            rospy.logdebug(
                "cur_arg: " + str(cur_arg) + " cur_slot_name: " + str(cur_slot) + " real slot content: " + str(
                    real_slot))
            if hasattr(real_slot, '_type'):
                rospy.logdebug("Recursive Object found in request/response of service call")
                already_set, counter = self.create_object_instance(already_set, real_slot, cur_slot, counter, inputs,
                                                                   object)
                object_counter += 1
            else:
                already_set.append(cur_slot)
                setattr(object, cur_slot, cur_arg)
                object_counter += 1
                counter += 1
                # sets the object as an attribute in the request were trying to build
        setattr(parent, name, object)
        return already_set, counter

    def recursive_delete_items(self, item):
        self.client.cancel_all_goals()
        for child in item.get_children():
            self.recursive_delete_items(child)
            self.server.server.delete_nodes([child])
        self.server.server.delete_nodes([self.result, self.result_node, self.goal_node, self.goal, self.parent])
        ros_server.own_rosnode_cleanup()

    def update_result(self, state, result):
        rospy.logdebug("updated result cb reached")
        self.status_node.set_value(map_status_to_string(state))
        self.result_node.set_value(repr(result))

    def update_state(self):
        rospy.logdebug("updated state cb reached")
        self.status_node.set_value(repr(self.client.get_goal_status_text()))

    def update_feedback(self, feedback):
        rospy.logdebug("updated feedback cb reached")
        self.message_callback(feedback)


def get_correct_name(topic_name):
    rospy.logdebug("getting correct name for: " + str(topic_name))
    splits = topic_name.split('/')
    return string.join(splits[0:-1], '/')


def getargarray(goal_class):
    array = []
    for slot_name in goal_class.__slots__:
        if slot_name != 'header':
            slot = getattr(goal_class, slot_name)
            if hasattr(slot, '_type'):
                array_to_merge = getargarray(slot)
                array.extend(array_to_merge)
            else:
                if isinstance(slot, list):
                    rospy.logdebug("Found an Array Argument!")
                    arg = ua.Argument()
                    arg.Name = slot_name
                    arg.DataType = ua.NodeId(ros_services.getobjectidfromtype("array"))
                    arg.ValueRank = -1
                    arg.ArrayDimensions = []
                    arg.Description = ua.LocalizedText("Array")
                else:
                    arg = ua.Argument()
                    if hasattr(goal_class, "__name__"):
                        arg.Name = goal_class.__name__ + slot_name
                    else:
                        arg.Name = slot_name
                    arg.DataType = ua.NodeId(ros_services.getobjectidfromtype(type(slot).__name__))
                    arg.ValueRank = -1
                    arg.ArrayDimensions = []
                    arg.Description = ua.LocalizedText(slot_name)
                array.append(arg)

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
            rospy.logdebug("deleting action: " + actionNameOPC)
            ros_server.own_rosnode_cleanup()
    for name in tobedeleted:
        del actionsdict[name]
