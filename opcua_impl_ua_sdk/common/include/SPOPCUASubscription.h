#ifndef __SPOPCUASUBSCRIPTION_H__
#define __SPOPCUASUBSCRIPTION_H__

#include <string>
#include <functional> 
#include "uasubscription.h"
#include "uasession.h"

using namespace UaClientSdk;

//typedef void (*SubscriptionCallback)(SPOPCUAValue value);

typedef std::function<void(SPOPCUAValue)> SubscriptionCallback;

class SPOPCUASubscription : public UaSubscriptionCallback
{

public:

	SPOPCUASubscription(UaSession* session);
	~SPOPCUASubscription();
	
	bool		Subscribe(std::string node_id, int namespace_index, SubscriptionCallback callback);
	bool		Unsubscribe();

	std::string GetNodeId();
	int			GetNamespaceIndex();

private:

	UaSession*				m_pSession;
    UaSubscription*			m_pSubscription;
	std::string				m_strNodeId;
	int						m_nNamespaceIndex;
	SubscriptionCallback	m_fCallback;

	// UaSubscriptionCallback implementation ----------------------------------------------------
    virtual void subscriptionStatusChanged(OpcUa_UInt32 clientSubscriptionHandle, const UaStatus&   status);
    virtual void dataChange(OpcUa_UInt32 clientSubscriptionHandle, const UaDataNotifications& dataNotifications, const UaDiagnosticInfos&   diagnosticInfos);
    virtual void newEvents(OpcUa_UInt32 clientSubscriptionHandle, UaEventFieldLists& eventFieldList);
    // UaSubscriptionCallback implementation ------------------------------------------------------

};

#endif
