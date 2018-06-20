#!/usr/bin/python
from opcua.common.ua_utils import get_nodes_of_namespace

from ros_global import *
from ros_messages import OpcUaROSMessage
from ros_services import OpcUaROSServiceNew
from ros_opc_ua import *

from datetime import datetime


class ROSServer(BasicROSServer):
    def create_messages(self):
        rospy.loginfo(' ----- start creating messages ------ ')
        message_object = OpcUaROSMessage(self.server, self._idx, self._idx_name)
        message_object.create_messages()
        self.server.load_type_definitions()
        self.message_object = message_object
        rospy.loginfo(' ----- %s messages created------ ' % str(len(message_object.created_data_types)))

    def export_messages(self):
        rospy.loginfo(' ----- start exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self.server, [self._idx])
        rospy.loginfo(' ----- %s nodes are to be exported ------ ' % len(node_to_export))
        self.server.export_xml(node_to_export, new_messageExportPath)
        rospy.loginfo(' ----- node message exported to %s ------ ' % new_messageExportPath)

    def create_service(self):
        rospy.loginfo(' ----- start creating services ------ ')
        service_object = OpcUaROSServiceNew(self.server, self._idx,
                                            self.message_object.created_data_types,
                                            self.message_object.created_variable_types)
        service_object.create_services()
        rospy.loginfo(' ----- %s services created------ ' % str(len(service_object.created_object_types.keys())))


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            # ua_server.create_messages()
            db = DictionaryBuilder(ua_server.server, ua_server._idx, 'ROSDictionary')
            db.create_data_type('StdMsgsHeader')
            dict_list = [item for item in ua_server.server.nodes.opc_binary.get_children()
                         if item.get_browse_name().NamespaceIndex != 0]
            dict_list[0].set_value(ua.Variant(b"""
                    <opc:TypeDictionary xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:tns="http://ros.org/rosopcua" DefaultByteOrder="LittleEndian" xmlns:opc="http://opcfoundation.org/BinarySchema/" xmlns:ua="http://opcfoundation.org/UA/" TargetNamespace="http://ros.org/rosopcua">
             <opc:Import Namespace="http://opcfoundation.org/UA/"/>
             <opc:StructuredType BaseType="ua:ExtensionObject" Name="StdMsgsHeader">
              <opc:Field TypeName="opc:UInt32" Name="seq"/>
              <opc:Field TypeName="opc:DateTime" Name="stamp"/>
              <opc:Field TypeName="opc:String" Name="frame_id"/>
             </opc:StructuredType>
            </opc:TypeDictionary>""", ua.VariantType.ByteString))
            ua_server.server.load_type_definitions()
            structure_folder = ua_server.server.nodes.base_structure_type
            data_type = structure_folder.get_child(ua.QualifiedName(to_camel_case('std_msgs/Header'),
                                                                    ua_server._idx))
            new_var = ua_server.server.nodes.objects.add_variable(nodeid_generator(ua_server._idx), 'testHeader',
                                                                  ua.Variant(None, ua.VariantType.Null),
                                                                  data_type.nodeid)
            msg = getattr(ua, to_camel_case('std_msgs/Header'))()
            msg.frame_id = 'test frame id'
            msg.seq = 20
            msg.stamp = datetime.now()
            new_var.set_value(msg)
            result = new_var.get_value()
            # ua_server.create_service()
            # ua_server.export_messages()
            rospy.spin()
    except Exception as e:
        print(e.message)
    # ua_server = ROSServer()
    # server.create_messages()
    # server.create_service()
    # server.export_messages()
    # db = DictionaryBuilder(ua_server.server, ua_server._idx)
    # db.create_data_type()
    # ua_server.server.load_type_definitions()
    # structure_folder = ua_server.server.nodes.base_structure_type
    # data_type = structure_folder.get_child(ua.QualifiedName(to_camel_case('std_msgs/Header'),
    #                                                         ua_server._idx))
    # new_var = ua_server.server.nodes.objects.add_variable(nodeid_generator(ua_server._idx), 'testHeader',
    #                                                       ua.Variant(None, ua.VariantType.Null),
    #                                                       data_type.nodeid)
    # msg = getattr(ua, to_camel_case('std_msgs/Header'))()
    # msg.frame_id = 'test frame id'
    # msg.seq = 9
    # msg.stamp = datetime.now()
    # new_var.set_value(msg)
    # result = new_var.get_value()
