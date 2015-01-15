// TODO: Would be cool to have dignostic topic to write subscirptions and stuff

#include <ros/ros.h>

#include <opcua_helpers.h>

#include "opcua_msgs/Address.h"
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
#include <opc/ua/subscription.h>

// Variables
OpcUa::UaClient _client;
std::map<std::string, std::unique_ptr<OpcUa::Subscription>> _subscriptions;
std::map<std::string, ros::Publisher> _callback_publishers;
std::map<std::string, uint32_t> _subscription_handles;

class SubClient : public OpcUa::SubscriptionHandler
{
  void DataChange(uint32_t handle, const OpcUa::Node& node, const OpcUa::Variant& value, OpcUa::AttributeID attr) const override
  {
    ROS_DEBUG("Callback....");
    _callback_publishers[OpcUa::ToString(node.GetId())].publish(fillTypeValue(value));
  }
};

// Variable
SubClient _sclt;

bool connect(opcua_srvs::Connect::Request &req, opcua_srvs::Connect::Response &res)
{
    // TODO: set connect status
    
    ROS_DEBUG("Establishing connection to OPC-UA server on address: %s", req.endpoint.c_str());
    try {
        _client.Connect(req.endpoint);
        ROS_INFO("Connection to OPC-UA server on address '%s' established!", req.endpoint.c_str());
        res.success = true;
    }
    catch (const std::exception& exc){
        ROS_ERROR("OPC-UA client node %s: Connection to OPC-UA server on address %s failed! Message: %s", ros::this_node::getName().c_str(), req.endpoint.c_str(), exc.what());
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Connection to OPC-UA server on address %s failed! Message: %s", req.endpoint.c_str(), exc.what());
        res.error_message = err_string;
    }
    catch (...) {
        ROS_ERROR("Connection to OPC-UA server on address %s failed with unknown exception", req.endpoint.c_str());
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
    ROS_DEBUG("OPC-UA client node %s: 'Read' service called with id_type: %s and namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);
    
    try {
        OpcUa::Node variable = _client.GetNode(convertAddressToNodeID(req.node));
        
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
        ROS_ERROR("OPC-UA client node %s: 'Read' service called with id_type: %s, namespace_index: %d, parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index, exc.what());
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Write service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Read' service called with id_type: %s, namespace_index: %d parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);
        
        res.success = false;
        res.error_message = "Unsubscribe service failed with Unknown exception";
    }    
    return true;
}

/////// Write
bool write(opcua_srvs::Write::Request &req, opcua_srvs::Write::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Write' service called with namespace_index: %d, id: %s,  type: '%s' parameters", ros::this_node::getName().c_str(), req.node.namespace_index, req.node.id_type.c_str(), req.data.type.c_str());
    
    try {
        OpcUa::Node variable = _client.GetNode(convertAddressToNodeID(req.node));
        
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
        ROS_ERROR("OPC-UA client node %s: 'Write' service called with id_type: %s, namespace_index: %d, type: '%s' parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index, req.data.type.c_str(), exc.what());
        
        res.success = false;
	char err_string;
	sprintf(&err_string, "Write service failed with exception: %s", exc.what());
	res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Write' service called with id_type: %s, namespace_index: %d, type: '%s' parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index, req.data.type.c_str());
        
        res.success = false;
        res.error_message = "Write service failed with Unknown exception";
    }
    
    return true;
}

// Method Call -- not implemented in freeopcua for now (2.1.2015)

// bool call_method(opcua_srvs::CallMethod::Request &req, opcua_srvs::CallMethod::Response &res)
// {
//     ROS_DEBUG("OPC-UA client node %s: 'CallMethod' service called with object_id: %s, id: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.object_id.c_str(), req.node.id_type.c_str(), req.node.namespace_index);
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
//         ROS_ERROR("OPC-UA client node %s: 'CallMethod' service called with object_id: %s, id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.object_id.c_str(), req.node.id_type.c_str(), req.node.namespace_index);
//     }
//     return false;
// }

// Subscriptions
// TODO can subscribe on only one node
bool subscribe(opcua_srvs::Subscribe::Request &req, opcua_srvs::Subscribe::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Subscribe' service called with id_type: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);
    
    try {
        // TODO: Check if already subscribed to node
        
        OpcUa::Node variable = _client.GetNode(convertAddressToNodeID(req.node));
        std::string node_string = OpcUa::ToString(variable.GetId());
        
        ros::NodeHandle nodeHandle("~");
        _callback_publishers[node_string] =  nodeHandle.advertise<opcua_msgs::TypeValue>(req.callback_topic, 1);
        
        _subscriptions[node_string] = _client.CreateSubscription(100, _sclt);
        _subscription_handles[node_string] =  _subscriptions[node_string]->SubscribeDataChange(variable);
        
        ROS_INFO("Node successfully subscribed!");        
        res.success = true;    
    }
    catch (const std::exception& exc){        
        ROS_ERROR("OPC-UA client node %s: 'Subscribe' service called with id_type: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Subscribe service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Subscribe' service called with id_type: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);
        
        res.success = false;
        res.error_message = "Unsubscribe service failed with Unknown exception";
    }
    
    return true;
}

bool unsubscribe(opcua_srvs::Unsubscribe::Request &req, opcua_srvs::Unsubscribe::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Unsubscribe' service called with id_type: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);

    try {
        // TODO: Check if already subscribed to node
        
        // TODO: Why is here return status false even the subscription was ok?
        
        OpcUa::Node variable = _client.GetNode(convertAddressToNodeID(req.node));        
        std::string node_string = OpcUa::ToString(variable.GetId());
        
        _subscriptions[node_string]->UnSubscribe(_subscription_handles[node_string]);
        _callback_publishers[node_string].shutdown();
        
        _subscriptions.erase(node_string);
        _subscription_handles.erase(node_string);
        _callback_publishers.erase(node_string);
        
        ROS_INFO("Node successfully unsubscribed!");
        res.success = true; 
    }
    catch (const std::exception& exc){        
       ROS_ERROR("OPC-UA client node %s: 'Unsubscribe' service called with id_type: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "Unsubscribe service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Unsubscribe' service called with id_type: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id_type.c_str(), req.node.namespace_index);
        
        res.success = false;
        res.error_message = "Unsubscribe service failed with Unknown exception";
    }
    
    return true;
}

/////// Main function

int main (int argc, char** argv)
{
    
    bool debug = true;
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