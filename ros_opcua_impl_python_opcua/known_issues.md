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