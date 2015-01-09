#include <ros/ros.h>

#include "opcua_msgs/TypeValue.h"
#include "opcua_srvs/Connect.h"
#include "opcua_srvs/Disconnect.h"

#include "opcua_client.h"

bool connect(opcua_srvs::Connect::Request &req, opcua_srvs::Connect::Response &res)
{
    ROS_DEBUG("Establishing connection to OPC-UA server on address: %s", req.server.c_str());
    try {
        _client.Connect(req.server);
        ROS_INFO("Connection to OPC-UA server on address '%s' established!", req.server.c_str());
        res.success = true;
    }
    catch (const std::exception& exc){
        ROS_ERROR("OPC-UA client node %s: Connection to OPC-UA server on address %s failed! Message: %s", ros::this_node::getName().c_str(), req.server.c_str(), exc.what());
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Connection to OPC-UA server on address %s failed! Message: %s", req.server.c_str(), exc.what());
        res.error_message = err_string;
    }
    catch (...) {
        ROS_ERROR("Connection to OPC-UA server on address %s failed with unknown exception", req.server.c_str());
        res.success = false;
        res.error_message = "'Connect' service failed with Unknown exception";
    }
    return true;
}

bool disconnect(opcua_srvs::Disconnect::Request &req, opcua_srvs::Disconnect::Response &res)
{
    ROS_DEBUG("Disconnecting from OPC-UA server...");
    try {
        _client.Disconnect();
        ROS_INFO("Disconnection succeded!");
    }
    catch (const std::exception& exc){
        ROS_ERROR("OPC-UA client node %s: Disconnection failed! (maybe client was not connected before?). Message: %s", ros::this_node::getName().c_str(), exc.what());
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Dissconect service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...) {
        ROS_ERROR("'Disconnect' service failed with unknown exception");
        res.success = false;
        res.error_message = "'Disconnect' service failed with unknown exception";
    }
    return true;
}

/////// Main function

int main (int argc, char** argv)
{
    
    bool debug = false;
    OpcUa::UaClient _client(debug);

    ros::init(argc, argv, "opcua_client_node");
    ros::NodeHandle nodeHandle("~");

    ros::ServiceServer connect_service = nodeHandle.advertiseService("connect", connect);
    ROS_INFO("OPC-UA client node %s: 'Connect' service available on on: %s", ros::this_node::getName().c_str(), connect_service.getService().c_str());
    ros::ServiceServer disconnect_service = nodeHandle.advertiseService("disconnect", disconnect);
    ROS_INFO("OPC-UA client node %s: 'Disconnect' service available on: %s", ros::this_node::getName().c_str(), disconnect_service.getService().c_str());

    ROS_INFO("OPCUA client node: %s is ready!", ros::this_node::getName().c_str());

    ros::spin();

    return 0;
}