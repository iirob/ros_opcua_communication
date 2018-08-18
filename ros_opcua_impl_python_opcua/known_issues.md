# Known Issues or problems

## Actions
* Wrong Argument Parsing, as Int32 type arguments are wrongly parsed to Int16 ua.Arguments()
(See BlinkyAction, PoliceAction on SR2 Robot in Lab for example)

* Move_Base_Simple action has special case as to fix Errors that were thrown, possibly it is a malformed Action Definition
```python
if 'move_base_simple' in self.name:
        self.goal_instance = self.goal_class()
    else:
        self.goal_instance = self.goal_class().goal
```

* Actions are seen as Actions when */goal*, */feedback*, */result* or */status* is in the topic name, better heuristic possibly needed
    + No heuristics found in action management of ROS, ideal in terminal were `rosaction list` analog to `rostopic list`.

* We ignore "header" in the goal call message instance to create our Arguments (and back the other way). Possible mistake See:
```python
while cur_slot == 'header':
                rospy.logdebug("ignoring header")
                object_counter += 1
                if object_counter < len(sample.__slots__):
                    cur_slot = sample.__slots__[object_counter]
```

## Topics:

* Better Idea for publishing OPC UA Data ?
    + Use Events to trigger while get a changed data. The client can subscribe the event and get informed.
    + Solved.

## Services

* Methods return value Strings are not correctly transfered to OPC UA somehow (maybe an UAExpert issue, as they can be printed in the call method onto the console correctly)
    + Try to write a small ua client, debugging there.
    + Solved.


## Modelling Problems

### High level python OPC UA does not support meta-modelling

The HasTypeDefinition reference is capsuled to BasicDataVariableType for Variable, therefore the created VariableType can not be used for further modelling.

Use the low level and middle level functions in python OPC UA to create functions for meta-modelling of ROS.

Solved.

## Modelling ROS in UA address space

### Static modelling

Possible information:

+ ROS message
ROS message can be easily retrieved with the ros build in tools, and will be modelled as variable types
+ ROS service
ROS services can be retrieved with tools, the structure is similar to ROS message, can be modelled as variable types, ROS service can be also modelled as object type, if a ROS node contains a specific service, an object out of this object type can be instantiated.
+ ROS topic & ROS action
    + Seem that no possible to know what kind of topics a node will publish and subscribe before it runs.
    + But topics and actions publish or subscribe messages defined in ROS messages.

Solved.

### Dynamic modelling

+ ROS node
ROS node can be retrieved from ROS master, which indicates the services and topics. Therefore the services and topics can be organized under ROS nodes, namely ROS node will be initiated in UA server as an object, its services will be provided as methods of the object and its interesting topics will be events or event notifier.

Solved.

## Method call

Problem: If python opcua supports method call with customized variable types as arguments (The ROS messages we imported).

Related discussion on GitHub [issue 297](https://github.com/FreeOpcUa/python-opcua/issues/297).

The discussion talked about method instantiation, whether a method belongs to an ObjectType or to an object, both are correct according to the discussion. In our case we want to instantiate a method, which maybe is a service or a topic pub/sub.
After instantiation, the arguments and the linked python methods are not set. we need to use the method in 'server.py' to relink the method.

```python
...
    def link_method(self, node, callback):
        """
        Link a python function to a UA method in the address space; required when a UA method has been imported
        to the address space via XML; the python executable must be linked manually
        Args:
            node: UA method node
            callback: python function that the UA method will call

        Returns:
        """
        self.iserver.isession.add_method_callback(node.nodeid, callback)
...
```

Since the current implementation of the python opcua is like this. My idea for ROS services is, create a big object type, which contains all services that can be listed by:

```bash
rossrv list
```

The methods inside this object type can not be called, or print always the meta data of the arguments defined by the ROS services.

When instantiate a ROS node, the services will be created as methods, which as nothing to do with the methods in the object type, and linked to correspondent python method.

This, however, does not solve the problem with the method arguments that contains customized variable types, which is now a problem if I do create methods with customized variables, this will cause a crash of the export xml function and problem of open the GUI of calling method in UAExpert.

Solved with new patches released for the xml export function.

## Extension object

also called customized structure

create extension objects on the fly,

use load_type_definitions when using extension objects in client side.

Extension object is defined actually in data type, data type must have encoding, encoding must be an object, this object id will be bind with a UA class, which is called extension object registration, and this encoding node will not be shown in address space. The encoding type must be a member of a DataTypeDictionaryType, the dictionary will be later used for information encoding and decoding.

Solved.

## Import xml instead of directly generate messages

Now the created extension objects can not be displayed correctly in UAExpert by importing xml, the reason is unknown, it seems that everything in direct generation and import are the same...

Besides, for a test case with 395 ros messages:

direct generation takes about 1.6s
xml import takes about 2.4s but works incorrectly

A ros parameter `import_xml_msgs` is imported to support both xml import and create on-the-fly.

## Actionlib uses unique goal ID to encrypt its topic publication

1. Actionlib uses unique goal ID to encrypt its topic publication. Problem of displaying the status of action client, if so, see [here](https://answers.ros.org/question/265723/actionlib-client-how-to-get-goal-id/)

2. Action client, how to model it?
    + Now only */goal*, */feedback*, */result* or */status* as property are tracked, and it seems that the */status* subscribed with standard status class `actionlib_msgs/GoalStatusArray` gives out empty result, to get the status, only by making a client in action and call method `client.get_state()`.
    + */feedback* and  */result* were modelled without publishing method, try to mock the action client does not make too much sense here.
    + For the */feedback*, */result*, there seems to be no value updates while the client exists, problem in 1?

3. Action server bug
    + At lease in action server `turtle_actionlib shape_server`, after shut down there is a leakage in unregistering the created topics (maybe, at least I did not see any de-registration in the source code of turtle_actionlib), there is always a warning `[WARN] [1534594441.388838]: Could not process inbound connection: [/rosopcua] is not a publisher of [/turtle1/pose]. Topics are [['/turtle1/cmd_vel', 'geometry_msgs/Twist'], ['/rosout', 'rosgraph_msgs/Log']]{'message_definition': 'float32 x\nfloat32 y\nfloat32 theta\n\nfloat32 linear_velocity\nfloat32 angular_velocity', 'callerid': '/rosopcua', 'tcp_nodelay': '0', 'md5sum': '863b248d5016ca62ea2e895ae5265cf9', 'topic': '/turtle1/pose', 'type': 'turtlesim/Pose'}`
    + In the rosnode, it seems that after the server started, it created a subscription to our ua_ros node `\rosopcua` , the reason is unknown, since it is not explict created by the code with `rospy.Subscriber()`, we can do nothing to remove it?
    + Possible solution? 