#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import math
import random
import sys
import time

import genpy
import rospy
import rosservice
from opcua import Server, ua

global server
global servicesDict


class OpcUaROSService:
    global server

    def __init__(self, parent, idx, service_name, service_class):
        self.name = service_name
        self._class = service_class
        self.proxy = rospy.ServiceProxy(self.name, self._class)
        self.counter = 0
        self._nodes = {}
        self.expressions = {}
        self._eval_locals = {}
        self._eval_locals = {}

        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']
        parent.add_method(ua.NodeId(self.name, idx), self.name, self.call_service, [], [])

    def call_service(self, *args):
        print ("reached callback")
        request = self._class._request_class()
        print (request)
        self.fill_message_slots(request, self.name, self.expressions, self.counter)
        print (request)
        try:
            response = self.proxy.call(request)
            return response
        except Exception as e:
            print(e)

    def fill_message_slots(self, message, topic_name, expressions, counter):
        print("Filling message slots!")
        if not hasattr(message, '__slots__'):
            return
        for slot_name in message.__slots__:
            slot_key = topic_name + '/' + slot_name

            # if no expression exists for this slot_key, continue with it's child slots
            if slot_key not in expressions:
                self.fill_message_slots(getattr(message, slot_name), slot_key, expressions, counter)
                continue

            expression = expressions[slot_key]
            if len(expression) == 0:
                continue

            # get slot type
            slot = getattr(message, slot_name)
            if hasattr(slot, '_type'):
                slot_type = slot._type
            else:
                slot_type = type(slot)

            self._eval_locals['i'] = counter
            value = self._evaluate_expression(expression, slot_type)
            if value is not None:
                setattr(message, slot_name, value)
            print(message)

    def _evaluate_expression(self, expression, slot_type):
        successful_eval = True
        successful_conversion = True

        try:
            # try to evaluate expression
            value = eval(expression, {}, self._eval_locals)
        except Exception:
            # just use expression-string as value
            value = expression
            successful_eval = False

        try:
            # try to convert value to right type
            value = slot_type(value)
        except Exception:
            successful_conversion = False

        if successful_conversion:
            return value
        elif successful_eval:
            print ("fill_message_slots(): can not convert expression to slot type: %s -> %s' % (type(value), slot_type)")
        else:
            print('fill_message_slots(): failed to evaluate expression: %s' % expression)

        return None


def refresh_services(idx, services_object_opc):
    global servicesDict

    rosservices = rosservice.get_service_list(include_nodes=False)

    for service_name_ros in rosservices:
        try:
            if service_name_ros not in servicesDict or servicesDict[service_name_ros] is None:
                service = OpcUaROSService(services_object_opc, idx, service_name_ros,
                                          rosservice.get_service_class_by_name(service_name_ros))
                servicesDict[service_name_ros] = service
        except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
            server.stop()

    for service_nameOPC in servicesDict:
        found = False
        for rosservice_name in rosservices:
            if service_nameOPC == rosservice_name:
                found = True
        if not found:
            servicesDict[service_nameOPC].recursive_delete_items(server.get_node(ua.NodeId(service_nameOPC, idx)))
            del servicesDict[service_nameOPC]


def main(args):
    global server
    global servicesDict

    rospy.init_node("opcua_server")
    servicesDict = {}
    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:21554/")
    server.set_server_name("ROS ua Server")

    server.start()  # setup our own namespace, this is expected
    uri = "http://ros.org"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our custom stuff
    objects = server.get_objects_node()

    servicesopc = objects.add_object(idx, "ROS-Services")

    while True:
        refresh_services(idx, servicesopc)
        time.sleep(2)
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
