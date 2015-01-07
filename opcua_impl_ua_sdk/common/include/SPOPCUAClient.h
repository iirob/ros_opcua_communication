#ifndef __SPOPCUACLIENT_H__
#define __SPOPCUACLIENT_H__

#include <string>
#include <list>
#include <map>

#include "uabase.h"
#include "uaclientsdk.h"

#include "SPOPCUATypes.h"
#include "SPOPCUASubscription.h"

using namespace UaClientSdk;


class SPOPCUAClient : public UaSessionCallback
{

public:

	SPOPCUAClient();
	~SPOPCUAClient();

	bool					Connect(std::string server_uri);
	bool					Disconnect();

	bool					ListResources(std::string node_id, int namespace_index, std::vector<std::string>& outputStringVector);
	bool					ListResourcesRefId(std::string node_id, int namespace_index, std::map<std::string, std::string>& outputMap);

	SPOPCUAValue			ReadValue(std::string resource_id, int namespace_index);
	bool					WriteValue(std::string resource_id, int namespace_index, SPOPCUAValue value);
	bool					CallMethod(std::string object_id, int object_namespace_index, std::string method_id, int method_namespace_index, std::vector<SPOPCUAValue>* arguments = 0);
	bool					SubscribeToNode(std::string node_id, int namespace_index, SubscriptionCallback callback);
	bool					UnsubscribeFromNode(std::string node_id, int namespace_index);

private:

    UaSession*  session;
	std::map<std::string,SPOPCUASubscription*>* subscriptions;

    bool			WriteInternal(const UaNodeIdArray& nodesToWrite, const UaVariantArray& valuesToWrite);
	UaDataValues	ReadInternal(const UaReadValueIds& nodesToRead);
	UaStatus		CallMethodInternal(const UaNodeId& objectNodeId, const UaNodeId& methodNodeId, std::vector<SPOPCUAValue>* arguments = 0);

	virtual void	connectionStatusChanged(OpcUa_UInt32 clientConnectionId, UaClient::ServerStatus serverStatus);

};

#endif