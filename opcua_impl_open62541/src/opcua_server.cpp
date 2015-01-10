// #define __cplusplus

// #include <ros/ros.h>

// provided by the open62541 lib
#include "ua_server.h"
#include "ua_namespace_0.h"

// provided by the user, implementations available in the /examples folder
#include "logger_stdout.h"
#include "networklayer_tcp.h"

UA_Boolean running = 1;

/////// Main function

int main (int argc, char** argv)
{    
//     ros::init(argc, argv, "opcua_server_node");
//     ros::NodeHandle nodeHandle("~");
    
    //configure server
    UA_Server *server = UA_Server_new();
    UA_Server_setServerCertificate(server, UA_STRING_NULL);
    UA_Server_addNetworkLayer(server, NetworkLayerTCP_new(UA_ConnectionConfig_standard, 16664));
    
    //add a node to the adresspace
    UA_Int32 *myInteger = UA_Int32_new();
    *myInteger = 42;
    UA_QualifiedName myIntegerName;
    UA_QUALIFIEDNAME_STATIC(myIntegerName, "the answer");
    UA_Server_addScalarVariableNode(server, &myIntegerName,
                                    myInteger, &UA_TYPES[UA_INT32],
                                    &UA_EXPANDEDNODEIDS[UA_OBJECTSFOLDER],
                                    &UA_NODEIDS[UA_ORGANIZES]);
    
    UA_ExpandedNodeId answerid;
    UA_ExpandedNodeId_init(&answerid);
    answerid.nodeId = (UA_NodeId){.identifierType = UA_NODEIDTYPE_NUMERIC, .identifier.numeric = 33, .namespaceIndex = 1};
    
    UA_Server_addScalarVariableNode(server, &myIntegerName,
                                    myInteger, &UA_TYPES[UA_INT32],
                                    &answerid,
                                    &UA_NODEIDS[UA_ORGANIZES]);
    
    
    UA_StatusCode retval = UA_Server_run(server, 1, &running);
    UA_Server_delete(server);

    return 0;
}