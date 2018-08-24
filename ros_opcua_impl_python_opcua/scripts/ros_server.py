#!/usr/bin/python
import time
import rospy

from opcua import ua

from ros_info_manage import ROSServiceManager, ROSTopicManager, ROSNodeManager, ROSParamManager, ROSInfoAgent
from basic_server_client import ROSBasicServer


class ROSServer(ROSBasicServer):

    def __init__(self):
        ROSBasicServer.__init__(self)
        self._root = self._server.nodes.objects
        self._msg_folder = self._root.add_folder(self._nodeid_generator(), 'rosmsg')
        self._srv_folder = self._root.add_folder(self._nodeid_generator(), 'rossrv')

        self._agent = ROSInfoAgent(self._ros_node_name, self._namespace_ros)

    def _load_messages(self):
        # FIXME: 1. bugs after xml import, extension object can not be used
        # 2. should compare in system to get correct msgs and srvs after xml loaded.
        if self._import_xml_msgs:
            self._type_list = self.import_messages()
        else:
            self.load_messages()

        self._server.load_type_definitions()
        self._create_static_info()

    def _create_static_info(self):
        """
        show rosmsg and rossrv in ua server
        :return:
        """
        for msg in self._msgs_dict:
            self._msg_folder.add_reference(self._msgs_dict[msg], ua.ObjectIds.Organizes)
        for srv in self._srvs_dict:
            self._srv_folder.add_reference(self._srvs_dict[srv], ua.ObjectIds.Organizes)

    def _initialize_info_managers(self):
        self._service_manager = ROSServiceManager(self._idx, self._root, self._type_dict)
        self._topic_manager = ROSTopicManager(self._idx, self._root, self._type_dict)

        service_ua_nodes = self._service_manager.get_node_dict()
        topic_status_nodes = self._topic_manager.get_status_ua_node()
        topic_publish_nodes = self._topic_manager.get_publish_ua_node()
        self._node_manager = ROSNodeManager(self._idx, self._root, self._type_dict, service_ua_nodes,
                                            topic_status_nodes, topic_publish_nodes)
        self._param_manager = ROSParamManager(self._idx, self._root)

    def _refresh_info(self):
        self._agent.node_cleanup()
        current_nodes, current_services, current_topics, current_params = self._agent.get_ros_info()
        self._service_manager.refresh_services(current_services)
        self._topic_manager.refresh_topics(current_topics)
        self._node_manager.refresh_nodes(current_nodes)
        self._param_manager.refresh_params(current_params)

    def _auto_refresh_info(self):
        while not rospy.is_shutdown():
            self._refresh_info()
            time.sleep(self._refresh_cycle_time)

    def initialize_server(self):
        self._load_messages()
        self._initialize_info_managers()
        self._start_server()

    def refresh(self):
        """
        For manual refresh please call this method
        :return:
        """
        if self._auto_refresh:
            self._auto_refresh_info()
        else:
            self._refresh_info()


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.initialize_server()
            ua_server.refresh()
    except Exception as e:
        print(e.message)
