#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

#include "opcua_msgs/TypeValue.h"

// #include "opcua_srvs/CallMethod.h"
#include "opcua_srvs/Connect.h"
#include "opcua_srvs/Disconnect.h"
#include "opcua_srvs/ListNode.h"
#include "opcua_srvs/Read.h"
#include "opcua_srvs/Subscribe.h"
#include "opcua_srvs/Write.h"
#include "opcua_srvs/Unsubscribe.h"

#include <opc/ua/client/client.h>
#include <opc/ua/node.h>
#include <opc/ua/subscription.h>

#include <iostream>
#include <stdexcept>
#include <thread>

OpcUa::UaClient _client;
std::unique_ptr<OpcUa::Subscription> _subscription;
ros::Publisher _callback_publisher;

// Commented values are not used in current implementation, there don't exist remapping to ROS
std::map<int, std::string> _TypeToStringMap = {
    
//     {0, "null"},
    {1, "bool"},
    {2, "int8"},
    {3, "uint8"},
    {4, "int16"},
    {5, "uint16"},
    {6, "int32"},
    {7, "uint32"},
    {8, "int64"},
    {9, "uint64"},
    {10, "float32"},
    {11, "float64"},
    {12, "string"},
//     {13, "date_time"},
//     {14, "guid"},
//     {15, "byte_string"},
//     {16, "xml_element"},
//     {17, "id"},
//     {18, "expanded_node_id"},
//     {19, "status_code"},
//     {20, "qualified_name"},
//     {21, "localized_text"},
//     {22, "extenstion_object"},
//     {23, "data_value"},
//     {24, "OpcUa::Variant"},
//     {25, "diagnostic_info"}
};

opcua_msgs::TypeValue fillTypeValue(const OpcUa::Variant& value) {
    
    opcua_msgs::TypeValue typeValue;
    
    typeValue.type = _TypeToStringMap[(int)value.Type()];
    
    if (typeValue.type == "bool") {
        typeValue.bool_d = (bool)value;
    }
    else if (typeValue.type == "int8") {
        typeValue.int8_d = (int8_t)value;
    }
    else if (typeValue.type == "uint8") {
        typeValue.uint8_d = (uint8_t)value;
    }
    else if (typeValue.type == "int16") {
        typeValue.int16_d = (int16_t)value;
    }
    else if (typeValue.type == "uint16") {
        typeValue.uint16_d = (uint16_t)value;
    }
    else if (typeValue.type == "int32") {
        typeValue.int32_d = (int32_t)value;
    }
    else if (typeValue.type == "uint32") {
        typeValue.uint32_d = (uint32_t)value;
    }
    else if (typeValue.type == "int64") {
        typeValue.int64_d = (int64_t)value;
    }
    else if (typeValue.type == "uint64") {
        typeValue.uint64_d = (uint64_t)value;
    }
    else if (typeValue.type == "float") {
        typeValue.float_d = (float)value;
    }
    else if (typeValue.type == "double") {
        typeValue.double_d = (double)value;
    }
    else if (typeValue.type == "string") {
        typeValue.string_d = std::string(value);
    }
    else {
        typeValue.type = "Unknown";
    }
    
    return typeValue;    
}

class SubClient : public OpcUa::SubscriptionHandler
{
  void DataChange(uint32_t handle, const OpcUa::Node& node, const OpcUa::Variant& value, OpcUa::AttributeID attr) const override
  {
    _callback_publisher.publish(fillTypeValue(value));
  }
};

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

/////// Read callbacks

bool read(opcua_srvs::Read::Request &req, opcua_srvs::Read::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Read' service called with id: %s and namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
    
    try {
        
        OpcUa::Node objects = _client.GetObjectsNode();
        std::vector<std::string> variable_path;
        boost::split(variable_path, req.node.id, boost::is_any_of(";"));     
        OpcUa::Node variable = objects.GetChild(variable_path);
        
        res.success = true;
        res.data = fillTypeValue(variable.GetValue());
        
        if (res.data.type == "Unknown") {
            res.success = false;
            char err_string;
            sprintf(&err_string, "Unknon data type!!");
            res.error_message = err_string;
            ROS_DEBUG("Reading failed!");
        }
    }
    catch (const std::exception& exc){        
        ROS_ERROR("OPC-UA client node %s: 'Read' service called with id: %s, namespace_index: %d, parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index, exc.what());
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Write service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Read' service called with id: %s, namespace_index: %d parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
        res.error_message = "Unsubscribe service failed with Unknown exception";
    }    
    return true;
}

/////// Write
bool write(opcua_srvs::Write::Request &req, opcua_srvs::Write::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Write' service called with namespace_index: %d, id: %s,  type: '%s' parameters", ros::this_node::getName().c_str(), req.node.namespace_index, req.node.id.c_str(), req.data.type.c_str());
    
    try {
        
        OpcUa::Node objects = _client.GetObjectsNode();        
        std::vector<std::string> variable_path;
        boost::split(variable_path, req.node.id, boost::is_any_of(";"));
        OpcUa::Node variable = objects.GetChild(variable_path);
        
        res.success = true;
        
        if (req.data.type == "bool") {
            variable.SetValue(OpcUa::Variant(req.data.bool_d));
        }
        else if (req.data.type == "int8") {
            variable.SetValue(OpcUa::Variant(req.data.int8_d));
        }
        else if (req.data.type == "uint8") {
            variable.SetValue(OpcUa::Variant(req.data.uint8_d));
        }
        else if (req.data.type == "int16") {
            variable.SetValue(OpcUa::Variant(req.data.int16_d));
        }
        else if (req.data.type == "uint16") {
            variable.SetValue(OpcUa::Variant(req.data.uint16_d));
        }
        else if (req.data.type == "int32") {
            variable.SetValue(OpcUa::Variant(req.data.int32_d));
        }
        else if (req.data.type == "uint32") {
            variable.SetValue(OpcUa::Variant(req.data.uint32_d));
        }
        else if (req.data.type == "int64") {
            variable.SetValue(OpcUa::Variant(req.data.int64_d));
        }
        else if (req.data.type == "uint64") {
            variable.SetValue(OpcUa::Variant(req.data.uint64_d));
        }
        else if (req.data.type == "float") {
            variable.SetValue(OpcUa::Variant(req.data.float_d));
        }
        else if (req.data.type == "double") {
            variable.SetValue(OpcUa::Variant(req.data.double_d));
        }
        else if (req.data.type == "string") {
            variable.SetValue(OpcUa::Variant(req.data.string_d));
        }
        else {
            res.success = false;
            char err_string;
            sprintf(&err_string, "Unknon data type %s", req.data.type.c_str());
            res.error_message = err_string;
            ROS_DEBUG("Writing failed!");
        }
    }
    catch (const std::exception& exc){        
        ROS_ERROR("OPC-UA client node %s: 'Write' service called with id: %s, namespace_index: %d, type: '%s' parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index, req.data.type.c_str(), exc.what());
        
        res.success = false;
	char err_string;
	sprintf(&err_string, "Write service failed with exception: %s", exc.what());
	res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Write' service called with id: %s, namespace_index: %d, type: '%s' parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index, req.data.type.c_str());
        
        res.success = false;
        res.error_message = "Write service failed with Unknown exception";
    }
    
    return true;
}

// Method Call -- not implemented in freeopcua for now (2.1.2015)

// bool call_method(opcua_srvs::CallMethod::Request &req, opcua_srvs::CallMethod::Response &res)
// {
//     ROS_DEBUG("OPC-UA client node %s: 'CallMethod' service called with object_id: %s, id: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.object_id.c_str(), req.node.id.c_str(), req.node.namespace_index);
// 
//     std::vector<SPOPCUAValue> arguments;
// 
//     for (std::vector<opcua_msgs::TypeValue>::const_iterator iterator = req.data.begin(); iterator != req.data.end(); ++iterator) {
//         arguments.push_back(*(new SPOPCUAValue((*iterator).type, (*iterator).value)));
//     }
// 
//     if (_client->CallMethod(req.object_id, req.node.namespace_index, req.node.id, req.node.namespace_index, &arguments)) {
//         ROS_DEBUG("Method called successfully!");
//         for (std::vector<SPOPCUAValue>::const_iterator iterator = arguments.begin(); iterator != arguments.end(); ++iterator) {
//             opcua_msgs::TypeValue typeValuePair;
//             SPOPCUAValue value = *iterator;
//             typeValuePair.type = value.GetTypeString();
//             typeValuePair.value = value.ToString();
//             res.data.push_back(typeValuePair);
//         }
//         return true;
//     }
//     else {
//         ROS_ERROR("OPC-UA client node %s: 'CallMethod' service called with object_id: %s, id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.object_id.c_str(), req.node.id.c_str(), req.node.namespace_index);
//     }
//     return false;
// }


// Subscriptions
// TODO can subscribe on only one node
bool subscribe(opcua_srvs::Subscribe::Request &req, opcua_srvs::Subscribe::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Subscribe' service called with id: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
    
    try {
    
        OpcUa::Node objects = _client.GetObjectsNode();  
        std::vector<std::string> variable_path;
        boost::split(variable_path, req.node.id, boost::is_any_of(";"));     
        OpcUa::Node variable = objects.GetChild(variable_path);
        
        SubClient sclt;
        _subscription = _client.CreateSubscription(100, sclt);
        _subscription->SubscribeDataChange(variable);
        
        ros::NodeHandle nodeHandle("~");
        _callback_publisher =  nodeHandle.advertise<opcua_msgs::TypeValue>(req.callback_topic, 1);
        ROS_DEBUG("Node successfully subscribed!");
        
        res.success = true;
    
    }
    catch (const std::exception& exc){        
        ROS_ERROR("OPC-UA client node %s: 'Subscribe' service called with id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Subscribe service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Subscribe' service called with id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
        res.error_message = "Unsubscribe service failed with Unknown exception";
    }
    
    return true;
}

bool unsubscribe(opcua_srvs::Unsubscribe::Request &req, opcua_srvs::Unsubscribe::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Unsubscribe' service called with id: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);

    try {
        // TODO here has to be done checking of unsubsribeing, currently can only subscribe on one Node
        
        _subscription->Delete();
        _callback_publisher.shutdown();
        ROS_DEBUG("Node successfully unsubscribed!");
        return true;    
    }
    catch (const std::exception& exc){        
       ROS_ERROR("OPC-UA client node %s: 'Unsubscribe' service called with id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
	char err_string;
	sprintf(&err_string, "Unsubscribe service failed with exception: %s", exc.what());
	res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Unsubscribe' service called with id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
	res.error_message = "Unsubscribe service failed with Unknown exception";
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
    
    // Reading of data
    ros::ServiceServer read_service = nodeHandle.advertiseService("read", read);
    ROS_INFO("OPC-UA client node %s: 'Read' service available on: %s", ros::this_node::getName().c_str(), read_service.getService().c_str());

    // Writing of data
    ros::ServiceServer write_service = nodeHandle.advertiseService("write", write);
    ROS_INFO("OPC-UA client node %s: 'Write' service available on: %s", ros::this_node::getName().c_str(), write_service.getService().c_str());

//     // Method Call
//     ros::ServiceServer call_method_service = nodeHandle.advertiseService("call_method", call_method);
//     ROS_INFO("OPC-UA client node %s: 'CallMethod' service available on: %s", ros::this_node::getName().c_str(), call_method_service.getService().c_str());

    // Subscriptions
    ros::ServiceServer subscribe_service = nodeHandle.advertiseService("subscribe", subscribe);
    ROS_INFO("OPC-UA client node %s: 'Subscribe' service available on: %s", ros::this_node::getName().c_str(), subscribe_service.getService().c_str());
    ros::ServiceServer unsubscribe_service = nodeHandle.advertiseService("unsubscribe", unsubscribe);
    ROS_INFO("OPC-UA client node %s: 'Unsubscribe' service available on: %s", ros::this_node::getName().c_str(), unsubscribe_service.getService().c_str());

  
    ROS_INFO("OPCUA client node: %s is ready!", ros::this_node::getName().c_str());

    ros::spin();

    return 0;
}