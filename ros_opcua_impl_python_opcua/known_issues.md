# Known Issues or problems

## ROS Topic deleting problem

- [ ] Error message: `[WARN] [1535063519.464253]: Inbound TCP/IP connection failed: connection from sender terminated before handshake header received. 0 bytes were received. Please check sender for additional details.`

Problem lies on ROS side, please check [here](https://github.com/Microsoft/WSL/issues/1391#issuecomment-300076193).

The [pull request](https://github.com/ros/ros_comm/pull/1050/files) to repair it in ros-lunar.

This problem happens sometimes, not always.

## Import xml instead of directly generate messages

- [ ] The created extension objects can not be displayed correctly in UAExpert by importing xml, the import function seem to have mixed OPC UA dictionary and data type nodes, the created data became one specific node.

Besides, for a test case with 395 ros messages:

direct generation takes about 1.6s
xml import takes about 2.4s but works incorrectly

A ros parameter `import_xml_msgs` is imported to support both xml import and create on-the-fly.
