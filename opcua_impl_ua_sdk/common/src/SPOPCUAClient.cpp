
#include <string>
#include <list>
#include <map>
#include <iostream>
#include <locale>
#include <sstream>

#include "SPOPCUAClient.h"
#include "SPOPCUATypes.h"
#include "SPOPCUASubscription.h"
#include "SPOPCUAAux.h"

#include "uaclientsdk.h"
#include "uasession.h"
#include "uaplatformlayer.h"

///////////////////////////////////////////////////////////////////////
SPOPCUAClient::SPOPCUAClient()
{
	// Initialize the UA Stack platform layer
    UaPlatformLayer::init();

	session = new UaSession();

	subscriptions = new std::map<std::string,SPOPCUASubscription*>();
}

///////////////////////////////////////////////////////////////////////
SPOPCUAClient::~SPOPCUAClient()
{
	this->Disconnect();

	delete subscriptions;

	// Cleanup the UA Stack platform layer
	UaPlatformLayer::cleanup();

	delete session;
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::Connect(std::string server_uri)
{
    UaStatus result;
    UaString sURL(server_uri.c_str());

    // Provide information about the client
    SessionConnectInfo sessionConnectInfo;
    UaString sNodeName("unknown_host");
    char szHostName[256];
    if (0 == UA_GetHostname(szHostName, 256))
    {
        sNodeName = szHostName;
    }
    sessionConnectInfo.sApplicationName = "SkillPro OPC-UA Client";
    // Use the host name to generate a unique application URI
    sessionConnectInfo.sApplicationUri  = UaString("urn:%1:UnifiedAutomation:GettingStartedClient").arg(sNodeName);
    sessionConnectInfo.sProductUri      = "urn:UnifiedAutomation:GettingStartedClient";
    sessionConnectInfo.sSessionName     = sessionConnectInfo.sApplicationUri;

    // Security settings are not initialized - we connect without security for now
    SessionSecurityInfo sessionSecurityInfo;

    result = session->connect(sURL, sessionConnectInfo, sessionSecurityInfo, this);

	// we can get the result error string by result.toString().toUtf8()
    return result.isGood();
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::Disconnect()
{
	if (session)
    {
		// unsubscribe from subscriptions
		std::map<std::string,SPOPCUASubscription*>::iterator it;
		for( it = subscriptions->begin(); it != subscriptions->end(); ++it )
		{
			it->second->Unsubscribe();
			delete it->second;
		}

        // disconnect if we're still connected
		UaStatus result;
        if (session->isConnected() != OpcUa_False)
        {
            ServiceSettings serviceSettings;
            result = session->disconnect(serviceSettings, OpcUa_True);
        }
        delete session;
        session = NULL;
		return result.isGood();
    }
	return false;
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::ListResources(std::string node_id, int namespace_index, std::vector<std::string>& outputStringVector)
{
    UaStatus result;

    // browse from root folder with no limitation of references to return
	UaNodeId nodeToBrowse;

	if( SPOPCUAAux::is_number(node_id ) )
		nodeToBrowse = UaNodeId(atoi(node_id.c_str()), namespace_index);
	else
		nodeToBrowse = UaNodeId(node_id.c_str(),namespace_index);

    ServiceSettings serviceSettings;
    BrowseContext browseContext;
    UaByteString continuationPoint;
    UaReferenceDescriptions referenceDescriptions;

    // configure browseContext
    browseContext.browseDirection = OpcUa_BrowseDirection_Forward;
    browseContext.referenceTypeId = OpcUaId_HierarchicalReferences;
    browseContext.includeSubtype = OpcUa_True;
    browseContext.maxReferencesToReturn = 0;

    //printf("\nBrowsing from Node %s...\n", nodeToBrowse.toXmlString().toUtf8());
    result = session->browse(
        serviceSettings,
        nodeToBrowse,
        browseContext,
        continuationPoint,
        referenceDescriptions);

    if (result.isGood())
    {
		for (unsigned int i=0; i<referenceDescriptions.length(); i++)
		{
			std::string result = "";
			result += "node: ";
			UaNodeId referenceTypeId(referenceDescriptions[i].ReferenceTypeId);
			result += ("[Ref=" + std::string(referenceTypeId.toString().toUtf8()) ) + "]";
			UaQualifiedName browseName(referenceDescriptions[i].BrowseName);
			result += std::string(browseName.toString().toUtf8()) + "( ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_Object) result += "Object ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_Variable) result += "Variable ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_Method) result += "Method ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_ObjectType) result +="ObjectType ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_VariableType) result += "VariableType ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_ReferenceType) result += "ReferenceType ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_DataType) result += "DataType ";
			if (referenceDescriptions[i].NodeClass & OpcUa_NodeClass_View) result += "View ";
			UaNodeId nodeId(referenceDescriptions[i].NodeId.NodeId);
			result += "[NodeId="+ std::string(nodeId.toFullString().toUtf8()) + "] ";
			result +=")\n";

			outputStringVector.push_back(result);
		}
		return true;
	}
	else
	{
		return false;
	}
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::ListResourcesRefId(std::string node_id, int namespace_index, std::map<std::string, std::string>& outputMap)
{
    UaStatus result;

    // browse from root folder with no limitation of references to return
	UaNodeId nodeToBrowse;

	if( SPOPCUAAux::is_number(node_id ) )
		nodeToBrowse = UaNodeId(atoi(node_id.c_str()), namespace_index);
	else
		nodeToBrowse = UaNodeId(node_id.c_str(),namespace_index);

    ServiceSettings serviceSettings;
    BrowseContext browseContext;
    UaByteString continuationPoint;
    UaReferenceDescriptions referenceDescriptions;

    // configure browseContext
    browseContext.browseDirection = OpcUa_BrowseDirection_Forward;
    browseContext.referenceTypeId = OpcUaId_HierarchicalReferences;
    browseContext.includeSubtype = OpcUa_True;
    browseContext.maxReferencesToReturn = 0;

    result = session->browse(
        serviceSettings,
        nodeToBrowse,
        browseContext,
        continuationPoint,
        referenceDescriptions);



    if (result.isGood())
    {
		for (unsigned int i=0; i<referenceDescriptions.length(); i++)
		{
			UaQualifiedName browseName(referenceDescriptions[i].BrowseName);
			std::string browsename = browseName.toString().toUtf8();
			UaNodeId nodeId(referenceDescriptions[i].NodeId.NodeId);
			std::string node_id = std::string(nodeId.toString().toUtf8());
			outputMap.insert(std::pair<std::string,std::string>(browsename,node_id));
		}
		return true;
	}
	else
	{
		return false;
	}
}


///////////////////////////////////////////////////////////////////////
SPOPCUAValue SPOPCUAClient::ReadValue(std::string resource_id, int namespace_index)
{
	// Configure one node to read
    UaReadValueIds    nodesToRead = SPOPCUAAux::BuildNodeToRead(resource_id, namespace_index);

	// just returns printable values (to be modified)
	UaDataValues values = ReadInternal(nodesToRead);

	if (OpcUa_IsGood(values[0].StatusCode))
	{
		return  SPOPCUAAux::BuildValueFromUaVariant( UaVariant(values[0].Value));
	}
	else
	{
		std::cout << "Error parsing value. StatusCode:" << values[0].StatusCode << std::endl;
		SPOPCUAValue none;
		return none;
	}
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::WriteValue(std::string resource_id, int namespace_index, SPOPCUAValue value)
{
	// Values to write (Type check incomplete)
	UaVariantArray valuesToWrite;
	valuesToWrite.create(1);
	UaVariant uavalue;
	uavalue = UaVariant( SPOPCUAAux::BuildVariantFromValue(value) );
	uavalue.copyTo(&valuesToWrite[0]);

	// Nodes to write
	UaNodeIdArray nodesToWrite = SPOPCUAAux::BuildNodeId(resource_id, namespace_index);

	// Do Writting
	return WriteInternal(nodesToWrite, valuesToWrite);
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::CallMethod(std::string object_id, int object_namespace_index, std::string method_id, int method_namespace_index, std::vector<SPOPCUAValue>* arguments)
{
	bool result = true;

    // call all methods from the configuration
    UaNodeIdArray objectNodeIds	= SPOPCUAAux::BuildNodeId(object_id,object_namespace_index);
    UaNodeIdArray methodNodeIds = SPOPCUAAux::BuildNodeId(method_id,method_namespace_index);

    for (OpcUa_UInt32 i = 0; i < methodNodeIds.length(); i++)
    {
        UaStatus stat = CallMethodInternal(objectNodeIds[i], methodNodeIds[i], arguments);
        if (stat.isNotGood())
        {
            result = false;
        }
    }

    return result;
}


///////////////////////////////////////////////////////////////////////
UaDataValues SPOPCUAClient::ReadInternal(const UaReadValueIds& nodesToRead)
{
	UaStatus          result;
    ServiceSettings   serviceSettings;
    UaDataValues      values;
    UaDiagnosticInfos diagnosticInfos;

    result = session->read(serviceSettings, 0, OpcUa_TimestampsToReturn_Both, nodesToRead, values, diagnosticInfos);

    if (result.isGood())
    {
		return values;
    }
    else
    {
        // Service call failed
        printf("Read failed with status %s\n", result.toString().toUtf8());
		return values;
	}
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::WriteInternal(const UaNodeIdArray& nodesToWrite, const UaVariantArray& valuesToWrite)
{
    UaStatus            result;
    ServiceSettings     serviceSettings;
    UaWriteValues       writeValues;
    UaStatusCodeArray   results;
    UaDiagnosticInfos   diagnosticInfos;

    // check in parameters
    if (nodesToWrite.length() != valuesToWrite.length())
    {
		return false;
        //return OpcUa_BadInvalidArgument;
    }

    // write all nodes from the configuration
    writeValues.create(nodesToWrite.length());

    for (OpcUa_UInt32 i = 0; i < writeValues.length(); i++)
    {
        writeValues[i].AttributeId = OpcUa_Attributes_Value;
        OpcUa_NodeId_CopyTo(&nodesToWrite[i], &writeValues[i].NodeId);
        // set value to write
        OpcUa_Variant_CopyTo(&valuesToWrite[i], &writeValues[i].Value.Value);
    }

//     printf("\nWriting...\n");
    result = session->write(
        serviceSettings,
        writeValues,
        results,
        diagnosticInfos);

	bool succeded_writing = true;

    if (result.isGood())
    {
        // Write service succeded - check individual status codes
        for (OpcUa_UInt32 i = 0; i < results.length(); i++)
        {
            if (!OpcUa_IsGood(results[i]))
            {
                succeded_writing = false;
                printf("Write failed for item[%d] with status %s\n", i, UaStatus(results[i]).toString().toUtf8());
            }
        }
    }
    else
    {
        // Service call failed
        printf("Write failed with status %s\n", result.toString().toUtf8());
    }

    return (result.isGood() && succeded_writing);
}

///////////////////////////////////////////////////////////////////////
UaStatus SPOPCUAClient::CallMethodInternal(const UaNodeId& objectNodeId, const UaNodeId& methodNodeId, std::vector<SPOPCUAValue>* arguments)
{
    UaStatus        result;
    ServiceSettings serviceSettings;
    CallIn          callRequest;
    CallOut         callResult;


    // NodeIds for Object and Method
    // we set no call parameters here
    callRequest.methodId = methodNodeId;
    callRequest.objectId = objectNodeId;

	// Copy method argument list into request
	if( arguments )
		SPOPCUAAux::CopyArgumentsToRequest( arguments, callRequest );

//     printf("\nCalling method %s on object %s, with %d arguments\n", methodNodeId.toString().toUtf8(), objectNodeId.toString().toUtf8(), callRequest.inputArguments.length());
    result = session->call(
        serviceSettings,
        callRequest,
        callResult);

    if (result.isGood())
    {
        // Call service succeded - check result
        if (callResult.callResult.isGood())
        {
//             printf("Call succeeded\n");
            arguments->clear();
            SPOPCUAAux::CopyArgumentsFromResult(callResult, arguments);
        }
        else
        {
            printf("Call failed with status %s\n", callResult.callResult.toString().toUtf8());
        }

    }
    else
    {
        // Service call failed
        printf("Call failed with status %s\n", result.toString().toUtf8());
    }

    return result;
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::SubscribeToNode(std::string node_id, int namespace_index,  SubscriptionCallback callback)
{
	SPOPCUASubscription* subscription = new SPOPCUASubscription(session);
	bool subs_success = subscription->Subscribe(node_id, namespace_index, callback);

	if( subs_success )
	{
		std::string map_id = SPOPCUAAux::BuildMapKey(node_id, namespace_index);
		subscriptions->insert( std::make_pair( map_id, subscription ) );
	}

	return subs_success;
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUAClient::UnsubscribeFromNode(std::string node_id, int namespace_index)
{
	std::string map_id = SPOPCUAAux::BuildMapKey(node_id, namespace_index);
	(subscriptions->at(map_id))->Unsubscribe();
	subscriptions->erase(map_id);
	return true;
}

///////////////////////////////////////////////////////////////////////
 void SPOPCUAClient::connectionStatusChanged(OpcUa_UInt32 clientConnectionId, UaClient::ServerStatus serverStatus)
 {

	 // Callback function when the connection status has changed

	OpcUa_ReferenceParameter(clientConnectionId);

	std:: cout << std::flush;

    switch (serverStatus)
    {
    case UaClient::Disconnected:
        std:: cout << "\n* Connection status changed to Disconnected *\n";
        break;
    case UaClient::Connected:
        std:: cout << "\n* Connection status changed to Connected *\n";
        break;
    case UaClient::ConnectionWarningWatchdogTimeout:
        std:: cout << "\n* Connection status changed to ConnectionWarningWatchdogTimeout *\n";
        break;
    case UaClient::ConnectionErrorApiReconnect:
        std:: cout << "\n* Connection status changed to ConnectionErrorApiReconnect *\n";
        break;
    case UaClient::ServerShutdown:
        std:: cout << "\n* Connection status changed to ServerShutdown *\n";
        break;
    case UaClient::NewSessionCreated:
        std:: cout << "\n* Connection status changed to NewSessionCreated *\n";
        break;
    }
 }

