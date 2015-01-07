#include <string>
#include <vector>
#include <locale>
#include <sstream>

#include "SPOPCUATypes.h"
#include "SPOPCUAAux.h"

#include "uabase.h"
#include "uaclientsdk.h"

///////////////////////////////////////////////////////////////////////
SPOPCUAValue SPOPCUAAux::BuildValueFromUaVariant(UaVariant variant)
{

	switch (variant.type()) {

		case OpcUaType_Int32:
			OpcUa_Int32 valueInt32;
			variant.toInt32(valueInt32);
			return *(new SPOPCUAValue((long)valueInt32));

		case OpcUaType_UInt32:
			OpcUa_UInt32 valueUInt32;
			variant.toUInt32(valueUInt32);
			return *(new SPOPCUAValue((unsigned long)valueUInt32));

		case OpcUaType_Int64:
			OpcUa_Int64 valueInt64;
			variant.toInt64(valueInt64);
			return *(new SPOPCUAValue((long long)valueInt64));

		case OpcUaType_UInt64:
			OpcUa_UInt64 valueUInt64;
			variant.toUInt64(valueUInt64);
			return *(new SPOPCUAValue((unsigned long long)valueUInt64));

		case OpcUaType_String:
			return *(new SPOPCUAValue(*(new std::string(variant.toString().toUtf8()))));

		case OpcUaType_Boolean:
			OpcUa_Boolean valueBool;
			variant.toBool(valueBool);
			return *(new SPOPCUAValue((bool)valueBool));

		case OpcUaType_NodeId:
		{
			UaNodeId valueNodeId;
			variant.toNodeId(valueNodeId);
			return *(new SPOPCUAValue(*(new std::string(valueNodeId.toFullString().toUtf8()))));
		}

		default:
			return *(new SPOPCUAValue());
	}
}

///////////////////////////////////////////////////////////////////////
UaVariant SPOPCUAAux::BuildVariantFromValue( SPOPCUAValue value )
{
	UaVariant variant;

	switch (value.GetType()) {

		case Int32:
			variant = UaVariant( (OpcUa_Int32)value.GetInt32() );
			break;

		case UInt32:
			variant = UaVariant( (OpcUa_UInt32)value.GetUInt32() );
			break;

		case Int64:
			variant = UaVariant( (OpcUa_Int64)value.GetInt64() );
			break;

		case UInt64:
			variant = UaVariant( (OpcUa_UInt64)value.GetUInt64() );
			break;

		case String:
			variant = UaVariant( UaString(value.GetString().c_str()) );
			break;

		case Boolean:
			variant = UaVariant( (OpcUa_Boolean)value.GetBoolean() );
			break;

		default:
			variant = UaVariant((OpcUa_Int32)-1);

	}

	return variant;
}

///////////////////////////////////////////////////////////////////////
UaReadValueIds SPOPCUAAux::BuildNodeToRead(std::string node_id, int namespace_index)
{
	UaReadValueIds return_node;
	UaObjectArray<UaNodeId> nodeIds;
	nodeIds.create(1);

	if( SPOPCUAAux::is_number(node_id ) )
		nodeIds[0].setNodeId(atoi(node_id.c_str()), namespace_index);
	else
		nodeIds[0].setNodeId(node_id.c_str(),namespace_index);

	// We do it with a loop so we can extend it to various nodes a time
	int count = nodeIds.length();
	return_node.create(count);
	for ( int i=0; i<count; i++ )
	{
		nodeIds[i].copyTo(&return_node[i].NodeId);
		return_node[i].AttributeId = OpcUa_Attributes_Value;
	}

	return return_node;
}

///////////////////////////////////////////////////////////////////////
UaNodeIdArray SPOPCUAAux::BuildNodeId(std::string node_id, int namespace_index)
{
	UaNodeIdArray nodeToWrite;
	nodeToWrite.create(1);
	UaNodeId node;
	if( SPOPCUAAux::is_number(node_id.c_str() ) )
		node.setNodeId(atoi(node_id.c_str()),namespace_index);
	else
		node.setNodeId(node_id.c_str(),namespace_index);
	node.copyTo(&nodeToWrite[0]);
	return nodeToWrite;
}

///////////////////////////////////////////////////////////////////////
void	SPOPCUAAux::CopyArgumentsToRequest(std::vector<SPOPCUAValue>* arguments, CallIn& request)
{
	// Copies all the method arguments to opc method call request
	request.inputArguments.create(arguments->size());
	for (unsigned int i = 0; i < arguments->size(); i++) {
		BuildVariantFromValue((*arguments)[i]).copyTo(&request.inputArguments[i]);
	}
}

///////////////////////////////////////////////////////////////////////
void	SPOPCUAAux::CopyArgumentsFromResult(CallOut& result, std::vector<SPOPCUAValue>* arguments)
{
	for (unsigned int i=0; i < result.outputArguments.length(); i++) {
		arguments->push_back(BuildValueFromUaVariant(*(new UaVariant(&result.outputArguments[i]))));
	}
}

///////////////////////////////////////////////////////////////////////
std::string SPOPCUAAux::BuildMapKey(std::string node_id, int namespace_index)
{
	std::stringstream out;
	out.str("");
	out << namespace_index;
	std::string map_id = node_id + "_" + out.str();
	return map_id;
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAAux::is_number(const std::string& s)
{
	std::locale loc;
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it,loc)) ++it;
    return !s.empty() && it == s.end();
}