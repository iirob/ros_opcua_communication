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

## Services
* Methods return value Strings are not correctly transfered to OPC UA somehow (maybe an UAExpert issue, as they can be printed in the call method onto the console correctly)

