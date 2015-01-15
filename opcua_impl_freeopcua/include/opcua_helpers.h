#include "opcua_msgs/Address.h"
#include "opcua_msgs/TypeValue.h"

#include <opc/ua/node.h>
#include <opc/ua/protocol/string_utils.h>

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

OpcUa::NodeID convertAddressToNodeID(const opcua_msgs::Address address) {
       
    if (address.id_type == "num") {
        return OpcUa::NumericNodeID(address.id_num, address.namespace_index);
    }
    else if (address.id_type == "str") {
        return OpcUa::StringNodeID(address.id_str, address.namespace_index);
    }
    else if (address.id_type == "guid") {
        return OpcUa::GuidNodeID(OpcUa::ToGuid(address.id_guid), address.namespace_index);
    }
    else {
        ROS_ERROR("OPC-UA client node %s: ID type %s does not exist!!", ros::this_node::getName().c_str(), address.id_type.c_str());
        throw std::exception();
    }
}

// OpcUa::NodeID convertAddressToString(const opcua_msgs::Address address) {
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