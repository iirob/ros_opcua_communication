/** @file
 * @brief       This file implements OpcUa client based on FreeOpcUa implementation.
 * @author      Denis Štogl
 *
 * @copyright   Copyright 2015 SkillPro Consortium
 *
 *              This file is part of SkillPro-Framework.
 *
 *              SkillPro-Framework is free software: you can redistribute it and/or modify
 *              it under the terms of the GNU Lesser General Public License as published by
 *              the Free Software Foundation, either version 3 of the License, or
 *              (at your option) any later version.
 *
 *              SkillPro-Framework is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              GNU Lesser General Public License for more details.
 *
 *              You should have received a copy of the GNU Lesser General Public License
 *              along with SkillPro-Framework. If not, see <http://www.gnu.org/licenses/>.
 *
 * @todo        Would be cool to have dignostic topic to write subscirptions and stuff
 */

#include <ros/ros.h>
#include <signal.h>

#include <opcua_helpers.h>

#include "opcua_msgs/Address.h"
#include "opcua_msgs/TypeValue.h"

#include "opcua_srvs/CallMethod.h"
#include "opcua_srvs/Connect.h"
#include "opcua_srvs/Disconnect.h"
#include "opcua_srvs/ListNode.h"
#include "opcua_srvs/Read.h"
#include "opcua_srvs/Subscribe.h"
#include "opcua_srvs/Write.h"
#include "opcua_srvs/Unsubscribe.h"

#include <opc/ua/client/client.h>
#include <opc/ua/subscription.h>

/// Client variable
OpcUa::UaClient _client;
/// Subscription list
/** Key is OpcUa string node identifier and value subscription reference.*/
std::map<std::string, std::unique_ptr<OpcUa::Subscription>> _subscriptions;
/// Callback publishers list
/** Key is OpcUa string node identifier and value ROS publisher */
std::map<std::string, ros::Publisher> _callback_publishers;
/// List of subscription handles
/** Key is OpcUa string node identifier and value is node handle. This is needed for deteting of subscriptions*/
std::map<std::string, uint32_t> _subscription_handles;
/// Connected status variable
bool _connected = false;

/// Subscription client class handles subscription callbacks.
/**
 * Currently is only implemented reaction on data change.
 */
class SubClient : public OpcUa::SubscriptionHandler
{
  /**
   * DataChange function is called by client internal mechanisms when change of data on subscribed node.
   * @param handle subscription handle
   * @param node node on which data were changed
   * @param value new value
   * @param attr OpcUa Attribute Id
   */
  void DataChange(uint32_t handle, const OpcUa::Node& node, const OpcUa::Variant& value, OpcUa::AttributeID attr) const override
  {
    ROS_DEBUG("Callback....");
    _callback_publishers[OpcUa::ToString(node.GetId())].publish(convertVariantToTypeValue(value));
  }
};

/// Subscription client store
/** @todo Need checking for multiple subscriptions */
SubClient _sclt;

/**
 * ROS-service call: Connect. Connects to OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
bool connect(opcua_srvs::Connect::Request &req, opcua_srvs::Connect::Response &res)
{
    // TODO: set connect status
    
    ROS_DEBUG("Establishing connection to OPC-UA server on address: %s", req.endpoint.c_str());
    try {
        _client.Connect(req.endpoint);
        ROS_INFO("Connection to OPC-UA server on address '%s' established!", req.endpoint.c_str());
        res.success = true;
        _connected = true;
    }
    catch (const std::exception& exc){
        ROS_ERROR("OPC-UA client node %s: Connection to OPC-UA server on address %s failed! Message: %s", ros::this_node::getName().c_str(), req.endpoint.c_str(), exc.what());
        
        res.success = false;
        char err_string[100];
        sprintf(err_string, "Connection to OPC-UA server on address %s failed! Message: %s", req.endpoint.c_str(), exc.what());
        res.error_message = err_string;
    }
    catch (...) {
        ROS_ERROR("Connection to OPC-UA server on address %s failed with unknown exception", req.endpoint.c_str());
        res.success = false;
        res.error_message = "'Connect' service failed with Unknown exception";
    }
    return true;
}

/**
 * ROS-service call: Disconnect. Disconnects from OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
bool disconnect(opcua_srvs::Disconnect::Request &req, opcua_srvs::Disconnect::Response &res)
{
    ROS_DEBUG("Disconnecting from OPC-UA server...");
    try {
        _client.Disconnect();
        ROS_INFO("Disconnection succeded!");
        _connected = false;
    }
    catch (const std::exception& exc){
        ROS_ERROR("OPC-UA client node %s: Disconnection failed! (maybe client was not connected before?). Message: %s", ros::this_node::getName().c_str(), exc.what());
        
        res.success = false;
        char err_string[100];
        sprintf(err_string, "Dissconect service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...) {
        ROS_ERROR("'Disconnect' service failed with unknown exception");
        res.success = false;
        res.error_message = "'Disconnect' service failed with unknown exception";
    }
    return true;
}

/**
 * ROS-service call: ListNode. Gets list of Node children from OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
bool list_node(opcua_srvs::ListNode::Request &req, opcua_srvs::ListNode::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'ListNode' service called with node Id: %s parameters", ros::this_node::getName().c_str(), req.node.nodeId.c_str());

    try {
      OpcUa::Node node = _client.GetNode(req.node.nodeId);

      for (OpcUa::Node child : node.GetChildren()){
          opcua_msgs::Address address;
          address.nodeId = OpcUa::ToString(child.GetId());
          address.qualifiedName = child.GetName().Name;
          res.children.push_back(address);
      }
      res.success = true;
    }
    catch (const std::exception& exc){
            ROS_ERROR("OPC-UA client node %s: 'ListNode' service called with node Id: %s, parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.nodeId.c_str(), exc.what());

            res.success = false;
            char err_string[100];
            sprintf(err_string, "ListNode service failed with exception: %s", exc.what());
            res.error_message = err_string;
        }
        catch (...)
        {
            ROS_ERROR("OPC-UA client node %s: 'ListNode' service called with node Id: %s parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.nodeId.c_str());

            res.success = false;
            res.error_message = "ListNode service failed with Unknown exception";
        }
        return true;
}

bool call_method(opcua_srvs::CallMethod::Request &req, opcua_srvs::CallMethod::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'CallMethod' service called with node Id: %s and %s parameters", ros::this_node::getName().c_str(), req.node.nodeId.c_str(), req.method.nodeId.c_str());

    try {
        OpcUa::Node node = _client.GetNode(req.node.nodeId);
        OpcUa::NodeID method_id = OpcUa::ToNodeID(req.method.nodeId);
        std::vector<OpcUa::Variant> inputArguments;

        for (opcua_msgs::TypeValue typeValue : req.data) {
            inputArguments.push_back(convertTypeValueToVariant(typeValue));
        }

        std::vector<OpcUa::Variant> outputArguments = node.CallMethod(method_id, inputArguments);

        for (OpcUa::Variant variant : outputArguments) {
            res.data.push_back(convertVariantToTypeValue(variant));
        }

        res.success = true;

    }
    catch (const std::exception& exc){
        ROS_ERROR("OPC-UA client node %s: 'CallMethod' service called with node Id: %s and %s parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.nodeId.c_str(), req.method.nodeId.c_str(), exc.what());

        res.success = false;
        char err_string[100];
        sprintf(err_string, "Call method service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'CallMethod' service called with node Id: %s parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.nodeId.c_str());

        res.success = false;
        res.error_message = "Call method service failed with Unknown exception";
    }
    return true;
}

/**
 * ROS-service call: Read. Reads value from OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
bool read(opcua_srvs::Read::Request &req, opcua_srvs::Read::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Read' service called with node Id: %s parameters", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
    
    try {
        OpcUa::Node variable = _client.GetNode(req.node.nodeId);
        
        res.success = true;
        res.data = convertVariantToTypeValue(variable.GetValue());
        
        if (res.data.type == "Unknown") {
            res.success = false;
            char err_string[100];
            sprintf(err_string, "Unknon data type!!");
            res.error_message = err_string;
            ROS_DEBUG("Reading failed!");
        }
    }
    catch (const std::exception& exc){        
        ROS_ERROR("OPC-UA client node %s: 'Read' service called with node Id: %s, parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.nodeId.c_str(), exc.what());
        
        res.success = false;
        char err_string[100];
        sprintf(err_string, "Read service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Read' service called with node Id: %s parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
        
        res.success = false;
        res.error_message = "Read service failed with Unknown exception";
    }    
    return true;
}

/**
 * ROS-service call: Write. Writes value to OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
bool write(opcua_srvs::Write::Request &req, opcua_srvs::Write::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Write' service called with node Id: %s, parameters", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
    
    try {
        OpcUa::Node variable = _client.GetNode(req.node.nodeId);
        
        OpcUa::Variant variant = convertTypeValueToVariant(req.data);

        if (!variant.IsNul()) {
          if (variant.Type() == OpcUa::VariantType::STRING) {
            if (((std::string)variant).length() == 0) {
              variant = std::string(" ");
            }
          }
          variable.SetValue(variant);
          res.success = true;
        }
        else {
            res.success = false;
            char err_string[100];
            sprintf(err_string, "Unknon data type %s", req.data.type.c_str());
            res.error_message = err_string;
            ROS_DEBUG("Writing failed!");
        }
    }
    catch (const std::exception& exc){        
        ROS_ERROR("OPC-UA client node %s: 'Write' service called with node Id: %s, type: '%s' parameters failed! Exception: %s", ros::this_node::getName().c_str(), req.node.nodeId.c_str(), req.data.type.c_str(), exc.what());
        
        res.success = false;
	char err_string[100];
	sprintf(err_string, "Write service failed with exception: %s", exc.what());
	res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Write' service called with node Id: %s, type: '%s' parameters failed! Unknown exception!", ros::this_node::getName().c_str(), req.node.nodeId.c_str(), req.data.type.c_str());
        
        res.success = false;
        res.error_message = "Write service failed with Unknown exception";
    }
    
    return true;
}

/**
 * ROS-service call: Subscribe. Subscribe to DataChange of Node on OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
/** @todo can subscribe on only one node */
bool subscribe(opcua_srvs::Subscribe::Request &req, opcua_srvs::Subscribe::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Subscribe' service called with node Id: %s parameters", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
    
    try {
        // TODO: Check if already subscribed to node
        
        OpcUa::Node variable = _client.GetNode(req.node.nodeId);
        std::string node_string = OpcUa::ToString(variable.GetId());
        
        ros::NodeHandle nodeHandle("~");
        _callback_publishers[node_string] =  nodeHandle.advertise<opcua_msgs::TypeValue>(req.callback_topic, 1);
        
        _subscriptions[node_string] = _client.CreateSubscription(100, _sclt);
        _subscription_handles[node_string] =  _subscriptions[node_string]->SubscribeDataChange(variable);
        
        ROS_INFO("Node successfully subscribed!");        
        res.success = true;    
    }
    catch (const std::exception& exc){        
        ROS_ERROR("OPC-UA client node %s: 'Subscribe' service called with node Id: %s parameters failed!", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
        
        res.success = false;
        char err_string[100];
        sprintf(err_string, "Subscribe service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Subscribe' service called with node Id: %s parameters failed!", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
        
        res.success = false;
        res.error_message = "Subscribe service failed with Unknown exception";
    }
    
    return true;
}

/**
 * ROS-service call: Unsubscribe. Unsubscribe from Node on OpcUa server.
 * @param req service request
 * @param res service response
 * @return True, if service execution broken than False.
 */
bool unsubscribe(opcua_srvs::Unsubscribe::Request &req, opcua_srvs::Unsubscribe::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'Unsubscribe' service called with node Id: %s parameters", ros::this_node::getName().c_str(), req.node.nodeId.c_str());

    try {
        // TODO: Check if already subscribed to node
        
        // TODO: Why is here return status false even the subscription was ok?
        
        OpcUa::Node variable = _client.GetNode(req.node.nodeId);
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
       ROS_ERROR("OPC-UA client node %s: 'Unsubscribe' service called with node Id: %s parameters failed!", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
        
        res.success = false;
        char err_string[100];
        sprintf(err_string, "Unsubscribe service failed with exception: %s", exc.what());
        res.error_message = err_string;
    }
    catch (...)
    {
        ROS_ERROR("OPC-UA client node %s: 'Unsubscribe' service called with node Id: %s parameters failed!", ros::this_node::getName().c_str(), req.node.nodeId.c_str());
        
        res.success = false;
        res.error_message = "Unsubscribe service failed with Unknown exception";
    }
    
    return true;
}

/**
 * Handler for Shutdown of the node
 */
void on_shutdown(int sig)
{
  _client.Disconnect();

  ros::shutdown();
}

/**
 * Start function of ROS-Node with default name 'opcua_client_node'
 * @param argc number of command line arguments
 * @param argv command line arguments
 * @return 0 if programs ends successfully.
 */
int main (int argc, char** argv)
{
    bool debug = true;
    OpcUa::UaClient _client(debug);

    ros::init(argc, argv, "opcua_client_node");
    ros::NodeHandle nodeHandle("~");
    signal(SIGINT, on_shutdown);

    ros::ServiceServer connect_service = nodeHandle.advertiseService("connect", connect);
    ROS_DEBUG("OPC-UA client node %s: 'Connect' service available on on: %s", ros::this_node::getName().c_str(), connect_service.getService().c_str());
    ros::ServiceServer disconnect_service = nodeHandle.advertiseService("disconnect", disconnect);
    ROS_DEBUG("OPC-UA client node %s: 'Disconnect' service available on: %s", ros::this_node::getName().c_str(), disconnect_service.getService().c_str());
    
    // List node
    ros::ServiceServer list_node_service = nodeHandle.advertiseService("list_node", list_node);
    ROS_DEBUG("OPC-UA client node %s: 'ListNode' service available on on: %s", ros::this_node::getName().c_str(), connect_service.getService().c_str());

     // Method Call
     ros::ServiceServer call_method_service = nodeHandle.advertiseService("call_method", call_method);
     ROS_DEBUG("OPC-UA client node %s: 'CallMethod' service available on: %s", ros::this_node::getName().c_str(), call_method_service.getService().c_str());

    // Reading of data
    ros::ServiceServer read_service = nodeHandle.advertiseService("read", read);
    ROS_DEBUG("OPC-UA client node %s: 'Read' service available on: %s", ros::this_node::getName().c_str(), read_service.getService().c_str());

    // Writing of data
    ros::ServiceServer write_service = nodeHandle.advertiseService("write", write);
    ROS_DEBUG("OPC-UA client node %s: 'Write' service available on: %s", ros::this_node::getName().c_str(), write_service.getService().c_str());

    // Subscriptions
    ros::ServiceServer subscribe_service = nodeHandle.advertiseService("subscribe", subscribe);
    ROS_DEBUG("OPC-UA client node %s: 'Subscribe' service available on: %s", ros::this_node::getName().c_str(), subscribe_service.getService().c_str());
    ros::ServiceServer unsubscribe_service = nodeHandle.advertiseService("unsubscribe", unsubscribe);
    ROS_DEBUG("OPC-UA client node %s: 'Unsubscribe' service available on: %s", ros::this_node::getName().c_str(), unsubscribe_service.getService().c_str());

  
    ROS_INFO("OPCUA client node: %s is ready!", ros::this_node::getName().c_str());

    ros::spin();

    return 0;
}
