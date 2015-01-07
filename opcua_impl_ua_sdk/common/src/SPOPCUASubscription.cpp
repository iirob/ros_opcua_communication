#include <string>
#include <iostream>
#include <functional>

#include "SPOPCUAClient.h"
#include "SPOPCUAAux.h"
#include "SPOPCUATypes.h"
#include "SPOPCUASubscription.h"

#include "uasubscription.h"
#include "uasession.h"

///////////////////////////////////////////////////////////////////////
SPOPCUASubscription::SPOPCUASubscription(UaSession* session)
{
	m_pSession = session;
}

///////////////////////////////////////////////////////////////////////
SPOPCUASubscription::~SPOPCUASubscription()
{

}

///////////////////////////////////////////////////////////////////////
bool SPOPCUASubscription::Subscribe(std::string node_id, int namespace_index, SubscriptionCallback callback)
{
	// Set callback function
	m_fCallback = callback;
	m_strNodeId = node_id;
	m_nNamespaceIndex = namespace_index;

	// We create a subscription and we monitorize one item (1 subscription for each item)
    UaStatus result_creation, result_monitoring;

    ServiceSettings serviceSettings;
    SubscriptionSettings subscriptionSettings;
    subscriptionSettings.publishingInterval = 100;

	// Create subscription
    result_creation = m_pSession->createSubscription(serviceSettings, this, 1,  subscriptionSettings, OpcUa_True, &m_pSubscription);

	// Add item to monitored items
    UaMonitoredItemCreateRequests itemsToCreate;
    UaMonitoredItemCreateResults createResults;
    itemsToCreate.create(1);
    itemsToCreate[0].ItemToMonitor.AttributeId = OpcUa_Attributes_EventNotifier;

	// Write node id and namespace
	itemsToCreate[0].ItemToMonitor.AttributeId = OpcUa_Attributes_Value;
	UaNodeIdArray node_id_array = SPOPCUAAux::BuildNodeId(node_id, namespace_index);
	OpcUa_NodeId_CopyTo(&node_id_array[0], &itemsToCreate[0].ItemToMonitor.NodeId);

    itemsToCreate[0].RequestedParameters.ClientHandle = 0;
    itemsToCreate[0].RequestedParameters.SamplingInterval = 0;
    itemsToCreate[0].RequestedParameters.QueueSize = 0;
    itemsToCreate[0].RequestedParameters.DiscardOldest = OpcUa_True;
    itemsToCreate[0].MonitoringMode = OpcUa_MonitoringMode_Reporting;

	result_monitoring = m_pSubscription->createMonitoredItems(serviceSettings, OpcUa_TimestampsToReturn_Both, itemsToCreate, createResults);

    if ( !result_creation.isGood() || !result_monitoring.isGood() )
    {
        m_pSubscription = NULL;
    }


	return ( result_creation.isGood() && result_monitoring.isGood() );
}

///////////////////////////////////////////////////////////////////////
bool SPOPCUASubscription::Unsubscribe()
{
    if ( m_pSubscription == NULL )
        return false;

    UaStatus result;
    ServiceSettings serviceSettings;

    result = m_pSession->deleteSubscription(serviceSettings, &m_pSubscription);

	m_fCallback = NULL;
    m_pSubscription = NULL;

    return result.isGood();
}

///////////////////////////////////////////////////////////////////////
std::string SPOPCUASubscription::GetNodeId()
{
	return m_strNodeId;
}

///////////////////////////////////////////////////////////////////////
int SPOPCUASubscription::GetNamespaceIndex()
{
	return m_nNamespaceIndex;
}

///////////////////////////////////////////////////////////////////////
void SPOPCUASubscription::subscriptionStatusChanged(OpcUa_UInt32 clientSubscriptionHandle, const UaStatus&   status)
{
	OpcUa_ReferenceParameter(clientSubscriptionHandle); // We use the callback only for this subscription
    if (status.isBad())
    {
        printf("Subscription not longer valid - failed with status %s\n", status.toString().toUtf8());
		// TODO: delete from parent subscription list
	}
}

///////////////////////////////////////////////////////////////////////
void SPOPCUASubscription::dataChange(OpcUa_UInt32 clientSubscriptionHandle, const UaDataNotifications& dataNotifications, const UaDiagnosticInfos&   diagnosticInfos)
{
	OpcUa_ReferenceParameter(clientSubscriptionHandle); // We use the callback only for this subscription
    OpcUa_ReferenceParameter(diagnosticInfos);
    OpcUa_UInt32 i = 0;

    for ( i=0; i<dataNotifications.length(); i++ )
    {
        if ( OpcUa_IsGood(dataNotifications[i].Value.StatusCode) )
        {
					m_fCallback(SPOPCUAAux::BuildValueFromUaVariant(dataNotifications[i].Value.Value));
				}
        else
        {
            UaStatus itemError(dataNotifications[i].Value.StatusCode);
            printf("  Variable = %d failed with status %s\n", dataNotifications[i].ClientHandle, itemError.toString().toUtf8());
        }
    }
}

///////////////////////////////////////////////////////////////////////
void SPOPCUASubscription::newEvents(OpcUa_UInt32 clientSubscriptionHandle, UaEventFieldLists& eventFieldList)
{
	OpcUa_UInt32 i = 0;
    printf("-- Event newEvents -----------------------------------------\n");
    printf("clientSubscriptionHandle %d \n", clientSubscriptionHandle);
    for ( i=0; i<eventFieldList.length(); i++ )
    {
        UaVariant message    = eventFieldList[i].EventFields[0];
        UaVariant sourceName = eventFieldList[i].EventFields[1];
        UaVariant severity   = eventFieldList[i].EventFields[2];
        printf("Event[%d] Message = %s SourceName = %s Severity = %s\n",
            i,
            message.toString().toUtf8(),
            sourceName.toString().toUtf8(),
            severity.toString().toUtf8());
    }
    printf("------------------------------------------------------------\n");
}