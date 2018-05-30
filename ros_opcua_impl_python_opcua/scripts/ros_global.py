# GLOBAL STATIC VARIABLES

# created UA nodes in UA Server, only the ROS related nodes
package_node_created = {}
# retrieved ROS package names
packages = []

# ros messages  'message' --> nodeVariableType
messageNode = {}

# ros_messages 'message' --> nodeDataType
dataTypeNode = {}

# ros Topics  'topic_name' --> 'topic_node'
topicNode = {}
messageExportPath = "message.xml"

# status string used in ros_action.py function _map_status_to_string
status_string = {9: "Goal LOST",
                 8: "Goal RECALLED",
                 7: "Goal RECALLING",
                 6: "Goal PREEMPTING",
                 5: "Goal REJECTED",
                 4: "Goal ABORTED",
                 3: "Goal SUCCEEDED",
                 2: "Goal PREEMPTED",
                 1: "Goal ACTIVE",
                 0: "Goal PENDING"}


# BaseDataType
# BaseDataVariableType

# baseDataVariableType_node = ;
