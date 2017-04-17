/** @file
 * @brief       This file contains helper functions for ROS implementation of OpcUa client.
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
 */

#include "ros_opcua_msgs/Address.h"
#include "ros_opcua_msgs/TypeValue.h"

#include <opc/ua/node.h>
#include <opc/ua/protocol/string_utils.h>

/// Remapping between ROS types and OpenOpcUa types
/** Commented values are not used in current implementation, there don't exist remapping to ROS.*/
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

/**
 * This function converts between ros_opcua_msgs::Address data type into OpcUa::NodeID data type.
 * @param address address to convert.
 * @return NodeID object created from address.
 */
//OpcUa::NodeID convertAddressToNodeID(const ros_opcua_msgs::Address address) {
//
//    if (address.id_type == "num") {
//        return OpcUa::NumericNodeID(address.id_num, address.namespace_index);
//    }
//    else if (address.id_type == "str") {
//        return OpcUa::StringNodeID(address.id_str, address.namespace_index);
//    }
//    else if (address.id_type == "guid") {
//        return OpcUa::GuidNodeID(OpcUa::ToGuid(address.id_guid), address.namespace_index);
//    }
//    else {
//        ROS_ERROR("OPC-UA client node %s: ID type %s does not exist!!", ros::this_node::getName().c_str(), address.id_type.c_str());
//        throw std::exception();
//    }
//}

/**
 * This function converts between OpcUa::NodeID data type into ros_opcua_msgs::Address data type.
 * @param nodeID nodeId to convert.
 * @return Address object created from nodeID.
 */
//ros_opcua_msgs::Address convertNodeIDToAddress(const OpcUa::NodeID nodeID) {
//
//    ros_opcua_msgs::Address address;
//    address.namespace_index = nodeID.GetNamespaceIndex();
//
//    if (nodeID.IsInteger()) {
//        address.id_type = "num";
//        address.id_num = nodeID.GetIntegerIdentifier();
//    }
//    else if (nodeID.IsString()) {
//        address.id_type = "str";
//        address.id_str = nodeID.GetStringIdentifier();
//    }
//    else if (nodeID.IsGuid()) {
//        address.id_type = "guid";
//        address.id_guid = OpcUa::ToString(nodeID.GetGuidIdentifier());
//    }
//    else {
//        ROS_ERROR("OPC-UA client node %s: ID type %s does not exist!!", ros::this_node::getName().c_str(), address.id_type.c_str());
//        throw std::exception();
//    }
//
//    return address;
//}

// OpcUa::NodeID convertAddressToString(const ros_opcua_msgs::Address address) {
//        
//     std::stringstream stream;
//     
//     stream << "ns=" << address.namespace_index << ";";
//     
//     if (address.id_type == "num") {
//         stream << "i=" << address.id_num << ";";
//     }
//     else if (address.id_type == "str") {
//         stream << "s=" << address.id_str << ";";
//     }
//     else if (address.id_type == "guid") {
//         stream << "g=" << address.id_guid << ";";
//     }
//     else {
//         ROS_ERROR("OPC-UA client node %s: ID type %s does not exist!!", ros::this_node::getName().c_str(), req.node.id_type.c_str());
//         throw std::exception("Read above!");
//     }
//     
//     return stream.str();
// }

/**
 * This function converts OpcUa::Variant data type into ros_opcua_msgs::TypeValue data type.
 * @param variant Variant to be converted.
 * @return TypeValue object created from variant.
 */
ros_opcua_msgs::TypeValue convertVariantToTypeValue(const OpcUa::Variant& variant) {
    
    ros_opcua_msgs::TypeValue typeValue;
    
    typeValue.type = _TypeToStringMap[(int)variant.Type()];
    
    if (typeValue.type == "bool") {
        typeValue.bool_d = (bool)variant;
    }
    else if (typeValue.type == "int8") {
        typeValue.int8_d = (int8_t)variant;
    }
    else if (typeValue.type == "uint8") {
        typeValue.uint8_d = (uint8_t)variant;
    }
    else if (typeValue.type == "int16") {
        typeValue.int16_d = (int16_t)variant;
    }
    else if (typeValue.type == "uint16") {
        typeValue.uint16_d = (uint16_t)variant;
    }
    else if (typeValue.type == "int32") {
        typeValue.int32_d = (int32_t)variant;
    }
    else if (typeValue.type == "uint32") {
        typeValue.uint32_d = (uint32_t)variant;
    }
    else if (typeValue.type == "int64") {
        typeValue.int64_d = (int64_t)variant;
    }
    else if (typeValue.type == "uint64") {
        typeValue.uint64_d = (uint64_t)variant;
    }
    else if (typeValue.type == "float" || typeValue.type == "float32") {
        typeValue.float_d = (float)variant;
    }
    else if (typeValue.type == "double" || typeValue.type == "float64") {
        typeValue.double_d = (double)variant;
    }
    else if (typeValue.type == "string") {
        typeValue.string_d = std::string(variant);
    }
    else {
        typeValue.type = "Unknown";
    }
    
    return typeValue;    
}

/**
 * This function converts ros_opcua_msgs::TypeValue data type into OpcUa::Variant data type.
 * @param typeValue TypeValue to be converted
 * @return Variant object created from typeValue
 */
OpcUa::Variant convertTypeValueToVariant(ros_opcua_msgs::TypeValue& typeValue)
{
    OpcUa::Variant variant;

    if (typeValue.type == "bool") {
        variant = bool(typeValue.bool_d);
    }
    else if (typeValue.type == "int8") {
        variant = typeValue.int8_d;
    }
    else if (typeValue.type == "uint8") {
        variant = typeValue.uint8_d;
    }
    else if (typeValue.type == "int16") {
        variant = typeValue.int16_d;
    }
    else if (typeValue.type == "uint16") {
        variant = typeValue.uint16_d;
    }
    else if (typeValue.type == "int32") {
        variant = typeValue.int32_d;
    }
    else if (typeValue.type == "uint32") {
        variant = typeValue.uint32_d;
    }
    else if (typeValue.type == "int64") {
        variant = typeValue.int64_d;
    }
    else if (typeValue.type == "uint64") {
        variant = typeValue.uint64_d;
    }
    else if (typeValue.type == "float" || typeValue.type == "float32") {
        variant = typeValue.float_d;
    }
    else if (typeValue.type == "double" || typeValue.type == "float64") {
        variant = typeValue.double_d;
    }
    else if (typeValue.type == "string") {
        variant = typeValue.string_d;
    }

    return variant;
}
