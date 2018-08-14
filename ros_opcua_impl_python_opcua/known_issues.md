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

## Services

* Methods return value Strings are not correctly transfered to OPC UA somehow (maybe an UAExpert issue, as they can be printed in the call method onto the console correctly)
    + Try to write a small ua client, debugging there.


## Modelling Problems

### High level python OPC UA does not support meta-modelling

The HasTypeDefinition reference is capsuled to BasicDataVariableType for Variable, therefore the created VariableType can not be used for further modelling.

Use the low level and middle level functions in python OPC UA to create functions for meta-modelling of ROS.

## Modelling ROS in UA address space

UA modelling has five key components, i.e. variable, property, variable type, object, object type. Normally property is used for representing simple variables, variable is used to create a instance from variable type, in a perspective of C++, see the example below:

```cpp
struct product {
  int weight;
  double price;
} apple, banana, melon;
```

Here the int, double are build in data types, weight, price are properties, the struct name product is a variable type, and the created instances apple, banana, melon are variables.

object and object type are two OO concepts, object type can be interpreted as class and object are instances of classes.

### Static modelling

Possible information:

+ ROS message
ROS message can be easily retrieved with the ros build in tools, and will be modelled as variable types
+ ROS service
ROS services can be retrieved with tools, the structure is similar to ROS message, can be modelled as variable types, ROS service can be also modelled as object type, if a ROS node contains a specific service, an object out of this object type can be instantiated.
+ ROS topic & ROS action
    + Seem that no possible to know what kind of topics a node will publish and subscribe before it runs.
    + But topics and actions publish or subscribe messages defined in ROS messages.

### Dynamic modelling
+ ROS node
ROS node can be retrieved from ROS master, which indicates the services and topics. Therefore the services and topics can be organized under ROS nodes, namely ROS node will be initiated in UA server as an object, its services will be provided as methods of the object and its interesting topics will be events or event notifier.

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

## Extension object

also called customized structure

create extension objects on the fly,

use load_type_definitions when using extension objects in client side.

Extension object is defined actually in data type, data type must have encoding, encoding must be an object, this object id will be bind with a UA class, which is called extension object registration, and this encoding node will not be shown in address space. The encoding type must be a member of a DataTypeDictionaryType, the dictionary will be later used for information encoding and decoding.


## Actionlib uses unique goal ID to encrypt its topic publication

Problem of displaying the status of action client, if so, see [here](https://answers.ros.org/question/265723/actionlib-client-how-to-get-goal-id/)

## Import xml instead of directly generate messages

Now the created extension objects can not be displayed correctly in UAExpert by importing xml, the reason is unknown, it seems that everything in direct generation and import are the same...

Besides, for a test case with 395 ros messages:

direct generation takes about 1.6s
xml import takes about 2.4s but works incorrectly
