#include "ua_transport_generated.h"
#include "ua_namespace_0.h"
#include "ua_util.h"

extern "C" {
        
}

class OpcUa::UaClient {
    
    void Connect(std::string endpoint) {
        
        
        
    }
}

typedef struct ConnectionInfo{
    UA_Int32 socket;
    UA_UInt32 channelId;
    UA_SequenceHeader sequenceHdr;
    UA_NodeId authenticationToken;
    UA_UInt32 tokenId;
} ConnectionInfo;

