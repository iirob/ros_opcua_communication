#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <opcua_ua_client_node.h>

#include "opcua_msgs/TypeValue.h"

#include "opcua_srvs/CallMethod.h"
#include "opcua_srvs/Connect.h"
#include "opcua_srvs/Disconnect.h"
#include "opcua_srvs/ListNode.h"
#include "opcua_srvs/Read.h"
#include "opcua_srvs/Subscribe.h"
#include "opcua_srvs/Write.h"
#include "opcua_srvs/Unsubscribe.h"


#include "SPOPCUAClient.h"
#include "SPOPCUATypes.h"

SPOPCUAClient* _client;
ros::Publisher _callback_publisher;

opcua_msgs::TypeValue fillTypeValue(const SPOPCUAValue value) {
    
    opcua_msgs::TypeValue typeValue;
    
    typeValue.type = value.GetTypeString();
    
    if (typeValue.type == "bool") {
        typeValue.bool_d = value.GetBool();
    }
    else if (typeValue.type == "int8") {
        typeValue.int8_d = value.GetInt8();
    }
    else if (typeValue.type == "uint8") {
        typeValue.uint8_d = value.GetUInt8();
    }
    else if (typeValue.type == "int16") {
        typeValue.int16_d = value.GetInt16();
    }
    else if (typeValue.type == "uint16") {
        typeValue.uint16_d = value.GetUInt16();
    }
    else if (typeValue.type == "int32") {
        typeValue.int32_d = value.GetInt32();
    }
    else if (typeValue.type == "uint32") {
        typeValue.uint32_d = value.GetUInt32();
    }
    else if (typeValue.type == "int64") {
        typeValue.int64_d = value.GetInt64();
    }
    else if (typeValue.type == "uint64") {
        typeValue.uint64_d = value.GetUInt64();
    }
    else if (typeValue.type == "float") {
        typeValue.float_d = value.GetFloat();
    }
    else if (typeValue.type == "double") {
        typeValue.double_d = value.GetDouble();
    }
    else if (typeValue.type == "string") {
        typeValue.string_d = value.GetString();
    }
    else {
        typeValue.type = "Unknown";
    }
    
    return typeValue;    
}

bool connect(opcua_srvs::Connect::Request &req, opcua_srvs::Connect::Response &res)
{
	ROS_DEBUG("Establishing connection to OPC-UA server on address: %s", req.server.c_str());
	if (_client->Connect(req.server)) {
		ROS_INFO("Connection to OPC-UA server on address '%s' established!", req.server.c_str());
		res.success = true;
	}
	else {
		ROS_ERROR("OPC-UA client node %s: Connection to OPC-UA server on address %s failed!", ros::this_node::getName().c_str(), req.server.c_str());
        res.success = false;
        char err_string;
        sprintf(&err_string, "Connection to OPC-UA server on address %s failed!", req.server.c_str());
        res.error_message = err_string;
	}
	return true;
}

bool disconnect(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_DEBUG("Disconnecting from OPC-UA server...");
	if (_client->Disconnect()) {
		ROS_INFO("Disconnection succeded!");
		res.success =  true;
	}
	else {
		ROS_ERROR("OPC-UA client node %s: Disconnection failed! (maybe client was not connected before?)", ros::this_node::getName().c_str());
        res.success = false;
        res.error_message = "'Disconnect' service failed!";
	}
	return true;
}

bool list_node(opcua_srvs::ListNode::Request &req, opcua_srvs::ListNode::Response &res)
{
	ROS_DEBUG("OPC-UA client node %s: 'ListNode' service called with node_id: %s and namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);

	std::vector<std::string> data;
	if (_client->ListResources(req.node.id, req.node.namespace_index, data)) {
		res.data = data;
        res.success = true;
	}
	else {
		ROS_ERROR("OPC-UA client node %s: 'ListNode' service: call to OPC-UA client failed!", ros::this_node::getName().c_str());
        
        res.success = false;
        res.error_message = "List node service failed with!";
	}
	return true;
}

/////// Read callbacks

bool read(opcua_srvs::Read::Request &req, opcua_srvs::Read::Response &res)
{
	ROS_DEBUG("OPC-UA client node %s: 'Read' service called with node_id: %s and namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);

    res.success = true;
    res.data = fillTypeValue(_client->ReadValue(req.node.id, req.node.namespace_index));
    
    if (res.data.type == "Unknown") {
        res.success = false;
        char err_string;
        sprintf(&err_string, "Unknon data type %s", res.data.type.c_str());
        res.error_message = err_string;
        ROS_DEBUG("Reading failed!");
    }

    return true;
}

/////// Write callbacks
bool write(opcua_srvs::Write::Request &req, opcua_srvs::Write::Response &res)
{
    ROS_DEBUG("OPC-UA client node %s: 'WriteString' service called with node_id: %s, namespace_index: %d and data: '%s' parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index, req.data.c_str());
    
    SPOPCUAValue* value;
    
    if (req.data.type == "bool") {
        value = new SPOPCUAValue(req.data.bool_d) ;
    }
    else if (req.data.type == "int8") {
        value = new SPOPCUAValue(req.data.int8_d);
    }
    else if (req.data.type == "uint8") {
        value = new SPOPCUAValue(req.data.uint8_d);
    }
    else if (req.data.type == "int16") {
        value = new SPOPCUAValue(req.data.int16_d);
    }
    else if (req.data.type == "uint16") {
        value = new SPOPCUAValue(req.data.uint16_d);
    }
    else if (req.data.type == "int32") {
        value = new SPOPCUAValue(req.data.int32_d);
    }
    else if (req.data.type == "uint32") {
        value = new SPOPCUAValue(req.data.uint32_d);
    }
    else if (req.data.type == "int64") {
        value = new SPOPCUAValue(req.data.int64_d);
    }
    else if (req.data.type == "uint64") {
        value = new SPOPCUAValue(req.data.uint64_d);
    }
    else if (req.data.type == "float") {
        value = new SPOPCUAValue(req.data.float_d);
    }
    else if (req.data.type == "double") {
        value = new SPOPCUAValue(req.data.double_d);
    }
    else if (req.data.type == "string") {
        value = new SPOPCUAValue(req.data.string_d);
    }
    else {
        res.success = false;
        char err_string;
        sprintf(&err_string, "Unknon data type %s", req.data.type.c_str());
        res.error_message = err_string;
        ROS_DEBUG("Writing failed!");
    }

    if (res.success == true) {
        if (_client->WriteValue(req.node.id, req.node.namespace_index, value)) {
            ROS_DEBUG("Writing succeeded!");
        }
        else {
            ROS_ERROR("OPC-UA client node %s: 'Write' service called with node_id: %s, namespace_index: %d' parameters failed!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
            
            res.success = false;
            res.error_message ="Writing failed!";
        }
    }
    return true;
}

// Method Call

bool call_method(opcua_srvs::CallMethod::Request &req, opcua_srvs::CallMethod::Response &res)
{
	ROS_DEBUG("OPC-UA client node %s: 'CallMethod' service called with object_id: %s, node_id: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.object_id.c_str(), req.node.id.c_str(), req.node.namespace_index);

	std::vector<SPOPCUAValue> arguments;

	for (std::vector<opcua_msgs::TypeValue>::const_iterator iterator = req.data.begin(); iterator != req.data.end(); ++iterator) {
		arguments.push_back(*(new SPOPCUAValue((*iterator).type, (*iterator).value)));
	}

	if (_client->CallMethod(req.object_id, req.node.namespace_index, req.node.id, req.node.namespace_index, &arguments)) {
		ROS_DEBUG("Method called successfully!");
		for (std::vector<SPOPCUAValue>::const_iterator iterator = arguments.begin(); iterator != arguments.end(); ++iterator) {
			res.data.push_back(fillTypeValue(*iterator));
		}
		res.success = true;
	}
	else {
		ROS_ERROR("OPC-UA client node %s: 'CallMethod' service called with object_id: %s, node_id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.object_id.c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
        res.error_message = "'CallMethod' service failed";
	}
	return true;
}

// Subscriptions

bool subscribe(opcua_srvs::Subscribe::Request &req, opcua_srvs::Subscribe::Response &res)
{
	ROS_DEBUG("OPC-UA client node %s: 'Subscribe' service called with node_id: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);

	if (_client->SubscribeToNode(req.node.id, req.node.namespace_index, callback)) {
		ros::NodeHandle nodeHandle("~");
		_callback_publisher =  nodeHandle.advertise<opcua_msgs::TypeValue>(req.callback_topic, 1);
		ROS_DEBUG("Node successfully subscribed!");
		res.success = true;
	}
	else {
		ROS_ERROR("OPC-UA client node %s: 'Subscribe' service called with node_id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "'Subscribe' service called with node_id: %s, namespace_index: %d parameters failed!", req.node.id.c_str(), req.node.namespace_index);
        res.error_message = err_string;
	}
	return true;
}

bool unsubscribe(opcua_srvs::Unsubscribe::Request &req, opcua_srvs::Unsubscribe::Response &res)
{
	ROS_DEBUG("OPC-UA client node %s: 'Unsubscribe' service called with node_id: %s, namespace_index: %d parameters", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);

	if (_client->UnsubscribeFromNode(req.node.id, req.node.namespace_index)) {
		_callback_publisher.shutdown();
		ROS_DEBUG("Node successfully unsubscribed!");
		res.success = true;
	}
	else {
		ROS_ERROR("OPC-UA client node %s: 'Unsubscribe' service called with node_id: %s, namespace_index: %d parameters failed!", ros::this_node::getName().c_str(), req.node.id.c_str(), req.node.namespace_index);
        
        res.success = false;
        char err_string;
        sprintf(&err_string, "'Unsubscribe' service called with node_id: %s, namespace_index: %d parameters failed!", req.node.id.c_str(), req.node.namespace_index);
        res.error_message = err_string;
	}
	return true;
}


// Callbackfunctions

void callback(SPOPCUAValue value)
{
    _callback_publisher.publish(fillTypeValue(value));
}


/////// Main function

int main (int argc, char** argv)
{
	_client = new SPOPCUAClient();

	ros::init(argc, argv, "opcua_client_node");
	ros::NodeHandle nodeHandle("~");

	ros::ServiceServer connect_service = nodeHandle.advertiseService("connect", connect);
	ROS_INFO("OPC-UA client node %s: 'Connect' service available on on: %s", ros::this_node::getName().c_str(), connect_service.getService().c_str());
	ros::ServiceServer disconnect_service = nodeHandle.advertiseService("disconnect", disconnect);
	ROS_INFO("OPC-UA client node %s: 'Disconnect' service available on: %s", ros::this_node::getName().c_str(), disconnect_service.getService().c_str());
	ros::ServiceServer list_node_service = nodeHandle.advertiseService("list_node", list_node);
	ROS_INFO("OPC-UA client node %s: 'ListNode' service available on: %s", ros::this_node::getName().c_str(), list_node_service.getService().c_str());

	// Reading of data
	ros::ServiceServer read_service = nodeHandle.advertiseService("read", read);
	ROS_INFO("OPC-UA client node %s: 'Read' service available on: %s", ros::this_node::getName().c_str(), read_service.getService().c_str());

	// Writing of data
	ros::ServiceServer write_service = nodeHandle.advertiseService("write", write);
	ROS_INFO("OPC-UA client node %s: 'Write' service available on: %s", ros::this_node::getName().c_str(), write_service.getService().c_str());

	// Method Call
	ros::ServiceServer call_method_service = nodeHandle.advertiseService("call_method", call_method);
	ROS_INFO("OPC-UA client node %s: 'CallMethod' service available on: %s", ros::this_node::getName().c_str(), call_method_service.getService().c_str());

	// Subscriptions
	ros::ServiceServer subscribe_service = nodeHandle.advertiseService("subscribe", subscribe);
	ROS_INFO("OPC-UA client node %s: 'Subscribe' service available on: %s", ros::this_node::getName().c_str(), subscribe_service.getService().c_str());
	ros::ServiceServer unsubscribe_service = nodeHandle.advertiseService("unsubscribe", unsubscribe);
	ROS_INFO("OPC-UA client node %s: 'Unsubscribe' service available on: %s", ros::this_node::getName().c_str(), unsubscribe_service.getService().c_str());


	ROS_INFO("OPC-UA client node: %s is ready!", ros::this_node::getName().c_str());

	ros::spin();

	return 0;
}