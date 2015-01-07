#ifndef __SPOPCUAAUX_H__
#define __SPOPCUAAUX_H__

#include <string>
#include <vector>
#include "SPOPCUATypes.h"

#include "uabase.h"
#include "uaclientsdk.h"

using namespace UaClientSdk;

class SPOPCUAAux
{
public:
	SPOPCUAAux();
	~SPOPCUAAux();
	static SPOPCUAValue	BuildValueFromUaVariant(UaVariant variant);
	static UaVariant	BuildVariantFromValue( SPOPCUAValue value );
	static UaReadValueIds	BuildNodeToRead(std::string node_id, int namespace_index);
	static UaNodeIdArray	BuildNodeId(std::string node_id, int namespace_index);
	static void	CopyArgumentsToRequest(std::vector<SPOPCUAValue>* arguments, CallIn& request);
	static void	CopyArgumentsFromResult(CallOut& result, std::vector<SPOPCUAValue>* arguments);
	static std::string	BuildMapKey(std::string node_id, int namespace_index);
	static bool 	is_number(const std::string& s);
};

#endif