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
 * @todo        Would be cool to have dignostic topic to write subscriptions and stuff
 */

#include <ros/ros.h>
#include <signal.h>

#include <opcua_helpers.h>

#include "ros_opcua_msgs/Address.h"
#include "ros_opcua_msgs/TypeValue.h"

#include "ros_opcua_srvs/CallMethod.h"
#include "ros_opcua_srvs/Connect.h"
#include "ros_opcua_srvs/Disconnect.h"
#include "ros_opcua_srvs/ListNode.h"
#include "ros_opcua_srvs/Read.h"
#include "ros_opcua_srvs/Subscribe.h"
#include "ros_opcua_srvs/Write.h"
#include "ros_opcua_srvs/Unsubscribe.h"

#include <opc/ua/client/client.h>
#include <opc/ua/subscription.h>

/// Client variable
OpcUa::UaClient _client(false);

std::unique_ptr<OpcUa::Subscription> subscription;
u_int32_t subscription_handle;

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
  void DataChange(uint32_t handle, const OpcUa::Node& node, const
OpcUa::Variant& value, OpcUa::AttributeId attr) override
  {
    ROS_DEBUG("Callback....");

    

  }
};

/// Subscription client store
/** @todo Need checking for multiple subscriptions */
SubClient _sclt;


/**
 * Handler for Shutdown of the node
 */
void on_shutdown(int sig)
{
    subscription->UnSubscribe(subscription_handle);
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
    ros::init(argc, argv, "variprog_opcua_client_node");
    ros::NodeHandle nodeHandle("~");
    signal(SIGINT, on_shutdown);
/*
    try {
        _client.Connect(endpoint);
        ROS_INFO("Connection to OPC-UA server on address '%s' established!", req.endpoint.c_str());
    }
    catch (const std::exception& exc){
        ROS_ERROR("OPC-UA client node %s: Connection to OPC-UA server on address %s failed! Message: %s", ros::this_node::getName().c_str(), req.endpoint.c_str(), exc.what());
        return 1;
    }
    catch (...) {
        ROS_ERROR("Connection to OPC-UA server on address %s failed with unknown exception", req.endpoint.c_str());
        return 1;
    }

//    OpcUa::Node variable = _client.GetNode(node.nodeId);

//    subscription = _client.CreateSubscription(100, _sclt);
//    subscription_handle = subscription->SubscribeDataChange(variable);

*/






    ROS_INFO("OPCUA client node: %s is ready!", ros::this_node::getName().c_str());

    ros::spin();

    return 0;
}
